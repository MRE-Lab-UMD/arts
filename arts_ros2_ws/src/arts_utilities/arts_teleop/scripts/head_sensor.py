#!/usr/bin/env python3
"""Head sensor node: GOOVIS HMD head presence -> /console/head_sensor (decision 18).

The GOOVIS G3 reports whether a head is in front of its optics as a USB HID report (vendor
880a, product 3501): the LAST byte of each report is 1 with a head present, anything else
without (collaborator A's reading of the device, used on it for the funder demo). This node
finds the matching /dev/hidraw* by its HID id, so the device number may change between
plugs, and publishes:

  /console/head_sensor   sensor_msgs/Joy, buttons = [present], on every change AND at
                         heartbeat_hz (default 20), stamped when the report was read. A
                         consumer treats a stale topic as "no head", which is why the
                         heartbeat exists: silence must never look like presence.

A lost or absent device publishes present = 0 and is retried every second. The device node
is root-only by default: deploy/modules/head_sensor.sh installs the udev rule.

This is the presence signal the da Vinci-style engagement uses (head in + handle aligned to
the instrument -> teleop may engage); the engagement logic itself is not here.
"""
import glob
import os
import select

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from sensor_msgs.msg import Joy

HID_ID = '0000880A:00003501'


def find_device():
    for uevent in sorted(glob.glob('/sys/class/hidraw/hidraw*/device/uevent')):
        try:
            with open(uevent) as f:
                if HID_ID in f.read().upper():
                    return '/dev/' + uevent.split('/')[4]
        except OSError:
            continue
    return None


class HeadSensor(Node):

    def __init__(self):
        super().__init__('head_sensor')
        self.period = 1.0 / self.declare_parameter('heartbeat_hz', 20.0).value
        self.pub = self.create_publisher(Joy, '/console/head_sensor', 10)
        self.fd = None
        self.present = 0
        self.warned = False
        self.next_open = 0.0
        self.last_pub = 0.0
        self.timer = self.create_timer(0.005, self._poll)

    def _open(self, now):
        if now < self.next_open:
            return
        self.next_open = now + 1.0
        path = find_device()
        if path is None:
            if not self.warned:
                self.get_logger().warn('GOOVIS (HID 880a:3501) not found; retrying every 1 s')
                self.warned = True
            return
        try:
            self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as e:
            if not self.warned:
                self.get_logger().error(f'cannot open {path}: {e} (udev rule installed? '
                                        'deploy/modules/head_sensor.sh)')
                self.warned = True
            return
        self.warned = False
        self.get_logger().info(f'GOOVIS head sensor on {path}')

    def _close(self, why):
        self.get_logger().warn(f'GOOVIS head sensor lost ({why}); head reported absent')
        try:
            os.close(self.fd)
        except OSError:
            pass
        self.fd = None
        self.present = 0

    def _poll(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        changed = False
        if self.fd is None:
            self._open(t)
        if self.fd is not None:
            while True:
                r, _, x = select.select([self.fd], [], [self.fd], 0.0)
                if x:
                    self._close('error')
                    changed = True
                    break
                if not r:
                    break
                try:
                    buf = os.read(self.fd, 64)
                except BlockingIOError:
                    break
                except OSError as e:
                    self._close(str(e))
                    changed = True
                    break
                if not buf:
                    self._close('EOF')
                    changed = True
                    break
                present = 1 if buf[-1] == 1 else 0
                if present != self.present:
                    self.present = present
                    changed = True
                    self.get_logger().info('head ' + ('IN' if present else 'OUT'))
        if changed or t - self.last_pub >= self.period:
            msg = Joy()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'head_sensor'
            msg.buttons = [self.present]
            self.pub.publish(msg)
            self.last_pub = t


def main():
    # SIGINT as KeyboardInterrupt: rclpy's own handler shuts the context down under spin()
    # and raised from inside the executor (seen on every Ctrl-C in the mock test).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = HeadSensor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.fd is not None:
            os.close(node.fd)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
