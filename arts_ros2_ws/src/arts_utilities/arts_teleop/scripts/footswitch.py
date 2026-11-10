#!/usr/bin/env python3
"""Footswitch node: Kinesis Savant Elite2 -> pedal state + mapped actions.

Reads the pedal's evdev keyboard interface directly, with an exclusive grab so presses never
type letters into whatever window has focus. Publishes:

  /console/footswitch_teleop    sensor_msgs/Joy, one message per press or release (auto-repeat
                         dropped), buttons in the order left, middle, right; stamped with the
                         KERNEL event time, i.e. when the pedal report arrived, not when this
                         node ran.
  /console/episode_mark  arts_interfaces/EpisodeMark START/END around each recorded segment.

Absolute names (decision 18), so the topics are the same wherever the node is started.

record_toggle records EPISODES: the first press starts one (START mark), the second ends it
(END mark); the next press starts a NEW episode with the next id. An episode is never resumed:
a training episode with a gap in it is not one. Mechanically it drives the image recorder's
pause/resume services (the recorder starts paused when launched with image_bag_paused:=true)
so the idle time between episodes is not written. Order matters because a paused recorder drops
messages: START is published after resume is confirmed, END before pause, with a short
settle so the recorder has written it. Both carry the press time as header.stamp.

The pedal mapping is config/input_mapping.yaml (decision 9); nothing is hardcoded here.
"""
import fcntl
import glob
import os
import queue
import select
import struct
import threading
import time

import rclpy
import yaml
from arts_interfaces.msg import EpisodeMark
from builtin_interfaces.msg import Time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosbag2_interfaces.srv import IsPaused, Pause, Resume
from sensor_msgs.msg import Joy

EVIOCGRAB = 0x40044590
EV_KEY = 1
EVENT = struct.Struct('llHHi')
PEDALS = ('left', 'middle', 'right')
END_SETTLE_S = 0.2


def stamp(sec, usec):
    return Time(sec=int(sec), nanosec=int(usec) * 1000)


class Footswitch(Node):

    def __init__(self):
        super().__init__('footswitch_teleop')
        mapping_file = self.declare_parameter('mapping_file', '').value
        self.recorder = self.declare_parameter('recorder', '/image_recorder').value
        self.label = self.declare_parameter('episode_label', '').value
        with open(mapping_file) as f:
            cfg = yaml.safe_load(f)['footswitch_teleop']
        self.device_glob = cfg['device']
        self.pedal_of = {int(k): v for k, v in cfg['pedals'].items()}
        self.action_of = dict(cfg['actions'])
        self.get_logger().info('mapping %s: %s' % (mapping_file, self.action_of))

        self.state = {p: 0 for p in PEDALS}
        self.joy_pub = self.create_publisher(Joy, '/console/footswitch_teleop', 10)
        self.mark_pub = self.create_publisher(EpisodeMark, '/console/episode_mark', 10)
        self.is_paused = self.create_client(IsPaused, self.recorder + '/is_paused')
        self.pause = self.create_client(Pause, self.recorder + '/pause')
        self.resume = self.create_client(Resume, self.recorder + '/resume')
        self.recording = None  # unknown until the recorder answers
        self.busy = False
        self.episode_id = 0

        self.events = queue.Queue()
        self.stop = threading.Event()
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.create_timer(0.01, self.drain)

    # --- device thread: open, grab, read; reopen after an unplug ---------------------------
    def read_loop(self):
        while not self.stop.is_set():
            paths = sorted(glob.glob(self.device_glob))
            if not paths:
                self.get_logger().warn('no pedal at %s; retrying' % self.device_glob,
                                       throttle_duration_sec=10.0)
                time.sleep(1.0)
                continue
            try:
                fd = os.open(paths[0], os.O_RDONLY | os.O_NONBLOCK)
            except OSError as e:
                self.get_logger().error('cannot open %s: %s (udev rule installed?)'
                                        % (paths[0], e), throttle_duration_sec=10.0)
                time.sleep(1.0)
                continue
            try:
                fcntl.ioctl(fd, EVIOCGRAB, 1)
                self.get_logger().info('pedal %s opened and grabbed' % paths[0])
                while not self.stop.is_set():
                    r, _, _ = select.select([fd], [], [], 0.2)
                    if not r:
                        continue
                    data = os.read(fd, EVENT.size * 64)
                    for i in range(0, len(data) - EVENT.size + 1, EVENT.size):
                        sec, usec, typ, code, value = EVENT.unpack_from(data, i)
                        # value 1 press, 0 release, 2 auto-repeat (dropped)
                        if typ == EV_KEY and value in (0, 1) and code in self.pedal_of:
                            self.events.put((sec, usec, self.pedal_of[code], value))
            except OSError as e:
                self.get_logger().error('pedal lost: %s; reopening' % e)
            finally:
                os.close(fd)
                # A pedal held at the unplug must not stay pressed.
                for p in PEDALS:
                    self.events.put((None, None, p, 0))

    # --- executor side ---------------------------------------------------------------------
    def drain(self):
        while not self.events.empty():
            sec, usec, pedal, value = self.events.get()
            if self.state[pedal] == value:
                continue
            self.state[pedal] = value
            t = stamp(sec, usec) if sec is not None else self.get_clock().now().to_msg()
            joy = Joy()
            joy.header.stamp = t
            joy.header.frame_id = 'footswitch_teleop'
            joy.buttons = [self.state[p] for p in PEDALS]
            self.joy_pub.publish(joy)
            if value == 1:
                self.on_press(pedal, t)

    def on_press(self, pedal, t):
        action = self.action_of.get(pedal)
        if action == 'record_toggle':
            self.record_toggle(t)
        else:
            self.get_logger().info('%s pedal: action %r not implemented yet' % (pedal, action))

    def record_toggle(self, t):
        if self.busy:
            self.get_logger().warn('record pedal ignored: previous toggle still in progress')
            return
        if not self.is_paused.service_is_ready():
            self.get_logger().error('%s not available: is the image recorder running?'
                                    % self.recorder)
            return
        self.busy = True
        self.is_paused.call_async(IsPaused.Request()).add_done_callback(
            lambda f: self.toggle_from(f, t))

    def toggle_from(self, future, t):
        paused = future.result().paused
        if paused:
            self.resume.call_async(Resume.Request()).add_done_callback(
                lambda f: self.started(t))
        else:
            self.publish_mark(EpisodeMark.END, t)
            self.create_oneshot(END_SETTLE_S, lambda: self.pause.call_async(
                Pause.Request()).add_done_callback(lambda f: self.stopped()))

    def started(self, t):
        self.episode_id += 1
        self.publish_mark(EpisodeMark.START, t)
        self.recording = True
        self.busy = False
        self.get_logger().info('RECORDING episode %d' % self.episode_id)

    def stopped(self):
        self.recording = False
        self.busy = False
        self.get_logger().info('SAVED episode %d (closed; the next press starts episode %d)'
                               % (self.episode_id, self.episode_id + 1))

    def publish_mark(self, mark_type, t):
        m = EpisodeMark()
        m.header.stamp = t
        m.mark_type = mark_type
        m.label = self.label
        m.episode_id = self.episode_id
        self.mark_pub.publish(m)

    def create_oneshot(self, delay_s, fn):
        holder = []

        def fire():
            holder[0].cancel()
            fn()
        holder.append(self.create_timer(delay_s, fire))

    def destroy_node(self):
        self.stop.set()
        self.reader.join(timeout=1.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = Footswitch()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
