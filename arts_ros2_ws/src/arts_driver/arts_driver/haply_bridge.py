#!/usr/bin/env python3
"""MTM bridge: Haply Inverse Service WebSocket -> the /mtm_left, /mtm_right topics (decisions 9, 18).

Connects to the Inverse Service bundled in Haply Hub 1.5.0 (service 3.5.7, ws://127.0.0.1:10001).
The service sends one frame per reply it receives, so this node sets the frame rate by pacing
its replies (rate_hz, default 500, decision 9); unpaced it ran at 3.5 kHz (2026-10-30). Every
message is stamped with the time its frame was RECEIVED here, before parsing.

  /mtm_<side>/cursor           geometry_msgs/PointStamped   handle point [m] ("cursor" is Haply's
                               term), frame mtm_<side>_base (the device's own base), every frame
  /mtm_<side>/cursor_velocity  geometry_msgs/Vector3Stamped [m/s], same frame, every frame
  /mtm_<side>/joint_states     sensor_msgs/JointState, mtm_<side>_joint_1..3: the Inverse3's three
                               motor angles [rad] and velocities [rad/s] (the service reports
                               DEGREES, -88.6 / 215.7 / -16.4 on 2026-11-01; converted here)
  /mtm_<side>/stylus_state     arts_interfaces/StylusState: VerseGrip orientation, buttons a/b/c,
                               grip (the service's raw Hall value), battery level
  /tf  mtm_<side>_base -> mtm_<side>_cursor: cursor position + the latest stylus orientation,
                               so RViz and Slicer can show the handle

Every frame repeats the LAST VerseGrip state, so stylus_state is published only when its
content changes: its rate is the pen's real data rate, not the frame rate (decision 9
amendment: never resample the VerseGrip). A still pen therefore publishes almost nothing.

Sides come from config/haply.yaml (device id -> left/right); an id not listed there is
logged once and ignored, and so is a device on a side left out of the `sides` parameter
(default both; arts_bringup sets it from mtm_left / mtm_right). Numbers arrive as JSON ints or floats (0 for 0.0) and are cast.
Each reply is a cursor force for each Inverse3 and an orientation probe for each VerseGrip
(the keep-alive collaborator A ran on the real devices). The force is ZERO unless
/mtm_<side>/cursor_force (geometry_msgs/Vector3Stamped, N, the device's own base frame; today
only camera_teleop's two-hand bar coupling sends it) arrived within force_timeout_s, judged by
its arrival here; a fresh one is capped at max_force_n in magnitude and a non-finite one is
dropped. So a sender that stops or dies leaves the handle free within force_timeout_s.
/mtm_<side>/cursor_position_target (geometry_msgs/PointStamped, m, device base frame; sent by
mtm_hold.py while the head is out) takes precedence while equally fresh: it goes out as the
service's set_cursor_position, whose position loop runs inside the service (~4 kHz, Haply docs
3.5.x), not through this PC. A force loop closed through ROS (300 N/m) went unstable on the real
handles 2026-11-06.

On every connection the node sets each Inverse3's device config over the service's HTTP API,
as collaborator A's node did: gravity compensation on with the handle mass from
config/haply.yaml (0.045 kg on both, decision 9), and home return off (A's setting). It reads
both back and logs what the service reports; a refused or unreachable request is logged as an
error and the node keeps publishing (the handle then feels heavier; nothing else changes).
"""
import json
import math
import threading
import time
import urllib.request

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, TransformStamped, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import websocket

from arts_interfaces.msg import StylusState

SIDES = ('left', 'right')


class HaplyBridge(Node):

    def __init__(self):
        super().__init__('mtm_bridge')
        self.uri = self.declare_parameter('uri', 'ws://127.0.0.1:10001').value
        self.period = 1.0 / self.declare_parameter('rate_hz', 500.0).value
        self.last_grip = {}
        # Which sides to publish (arts_bringup's mtm_left / mtm_right switches); a device on a
        # side not listed is treated like an unlisted id: logged once and ignored.
        self.sides = [s for s in self.declare_parameter('sides', list(SIDES)).value if s]
        for s in self.sides:
            if s not in SIDES:
                raise ValueError(f'sides: {s!r} is not left or right')
        inv_ids = self.declare_parameter('inverse3_ids', ['']).value
        inv_sides = self.declare_parameter('inverse3_sides', ['']).value
        grip_ids = self.declare_parameter('versegrip_ids', ['']).value
        grip_sides = self.declare_parameter('versegrip_sides', ['']).value
        self.inv_side = self._pairs(inv_ids, inv_sides, 'inverse3')
        self.grip_side = self._pairs(grip_ids, grip_sides, 'versegrip')
        self.unknown = set()
        masses = self.declare_parameter('gravity_compensation_kg', [0.0]).value
        if len(masses) != len(inv_ids):
            raise ValueError('gravity_compensation_kg: one value per inverse3_ids entry')
        self.mass = {i: float(m) for i, m in zip(inv_ids, masses) if i in self.inv_side}
        self.home_return = self.declare_parameter('home_return', False).value
        self.http = self.uri.replace('ws://', 'http://', 1).rstrip('/')
        self.max_force = self.declare_parameter('max_force_n', 3.0).value
        self.force_timeout = self.declare_parameter('force_timeout_s', 0.05).value
        self.force = {}   # side -> (monotonic arrival time, [x, y, z] N)
        self.target = {}  # side -> (monotonic arrival time, [x, y, z] m)

        self.pub_pos, self.pub_vel, self.pub_js, self.pub_stylus = {}, {}, {}, {}
        self.orientation = {}   # latest stylus orientation per side, for the cursor tf frame
        for side in set(self.inv_side.values()):
            ns = f'/mtm_{side}'
            self.pub_pos[side] = self.create_publisher(PointStamped, f'{ns}/cursor', 10)
            self.pub_vel[side] = self.create_publisher(Vector3Stamped, f'{ns}/cursor_velocity', 10)
            self.pub_js[side] = self.create_publisher(JointState, f'{ns}/joint_states', 10)
            self.create_subscription(Vector3Stamped, f'{ns}/cursor_force',
                                     lambda m, side=side: self._on_force(side, m), 10)
            self.create_subscription(PointStamped, f'{ns}/cursor_position_target',
                                     lambda m, side=side: self._on_target(side, m), 10)
        for side in set(self.grip_side.values()):
            self.pub_stylus[side] = self.create_publisher(
                StylusState, f'/mtm_{side}/stylus_state', 10)
        self.tf = TransformBroadcaster(self)

        self.probe = [{'device_id': i, 'commands': {'probe_orientation': {}}}
                      for i in self.grip_side]
        self.get_logger().info(f'{self.uri}: inverse3 {self.inv_side}, versegrip {self.grip_side}, '
                               f'cursor force cap {self.max_force} N, timeout {self.force_timeout} s')

        self.running = True
        self.ws = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _on_force(self, side, m):
        f = [m.vector.x, m.vector.y, m.vector.z]
        if all(math.isfinite(x) for x in f):
            self.force[side] = (time.monotonic(), f)

    def _on_target(self, side, m):
        p = [m.point.x, m.point.y, m.point.z]
        if all(math.isfinite(x) for x in p):
            self.target[side] = (time.monotonic(), p)

    def _reply(self):
        now = time.monotonic()
        inv = []
        for i, side in self.inv_side.items():
            t_p = self.target.get(side)
            if t_p is not None and now - t_p[0] <= self.force_timeout:
                p = t_p[1]
                inv.append({'device_id': i, 'commands': {'set_cursor_position': {
                    'position': {'x': p[0], 'y': p[1], 'z': p[2]}}}})
                continue
            f = [0.0, 0.0, 0.0]
            t_f = self.force.get(side)
            if t_f is not None and now - t_f[0] <= self.force_timeout:
                n = math.sqrt(sum(x * x for x in t_f[1]))
                k = 1.0 if n <= self.max_force else self.max_force / n
                f = [x * k for x in t_f[1]]
            inv.append({'device_id': i, 'commands': {'set_cursor_force': {
                'vector': {'x': f[0], 'y': f[1], 'z': f[2]}}}})
        return json.dumps({'inverse3': inv, 'wireless_verse_grip': self.probe})

    def _pairs(self, ids, sides, what):
        out = {}
        for i, s in zip(ids, sides):
            if not i:
                continue
            if s not in SIDES:
                raise ValueError(f'{what} {i}: side must be left or right, not {s!r}')
            if s in self.sides:
                out[i] = s
        return out

    def _run(self):
        delay = 1.0
        while self.running and rclpy.ok():
            try:
                self.ws = websocket.create_connection(self.uri, timeout=2.0)
                self.get_logger().info('connected')
                self._configure()
                delay = 1.0
                next_send = time.monotonic()
                while self.running:
                    msg = self.ws.recv()
                    t = self.get_clock().now().to_msg()  # receive time, before parsing
                    if not msg:
                        continue
                    self._frame(json.loads(msg), t)
                    next_send += self.period
                    wait = next_send - time.monotonic()
                    if wait > 0:
                        time.sleep(wait)
                    else:
                        next_send = time.monotonic()  # fell behind: no burst to catch up
                    self.ws.send(self._reply())
            except Exception as e:  # noqa: B902 -- any failure: log, back off, reconnect
                if self.running:
                    self.get_logger().warn(f'{type(e).__name__}: {e}; retry in {delay:.0f} s')
                    time.sleep(delay)
                    delay = min(delay * 2, 5.0)
            finally:
                if self.ws is not None:
                    self.ws.close()
                    self.ws = None

    def _config(self, dev, key, body=None):
        """POST body to (or, without one, GET) /inverse3/<dev>/config/<key>; returns its data."""
        req = urllib.request.Request(
            f'{self.http}/inverse3/{dev}/config/{key}',
            data=None if body is None else json.dumps(body).encode(),
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=2.0) as r:
            j = json.loads(r.read())
        if not j.get('ok', False):
            raise RuntimeError(j.get('error') or 'refused')
        return j.get('data')

    def _configure(self):
        for dev, side in self.inv_side.items():
            try:
                # The service answers ok to a POST even for a device id it does not have
                # (2026-11-03), so only the read-back counts.
                want_g = {'enabled': True, 'scaling_factor': self.mass[dev]}
                want_h = {'enabled': bool(self.home_return)}
                self._config(dev, 'gravity_compensation', want_g)
                self._config(dev, 'home_return', want_h)
                g = self._config(dev, 'gravity_compensation')
                h = self._config(dev, 'home_return')
                if (g.get('enabled') is not True
                        or abs(float(g.get('scaling_factor', -1)) - self.mass[dev]) > 1e-6
                        or h.get('enabled') != want_h['enabled']):
                    raise RuntimeError(f'read back gravity compensation {g}, home return {h}')
                self.get_logger().info(
                    f'inverse3 {dev} ({side}): gravity compensation {g["scaling_factor"]} kg, '
                    f'home return {"on" if h["enabled"] else "off"} (read back)')
            except Exception as e:  # noqa: B902 -- the stream does not depend on it
                self.get_logger().error(
                    f'inverse3 {dev} ({side}): device config not set ({type(e).__name__}: {e}); '
                    'publishing without it')

    def _side(self, table, dev, what):
        i = dev.get('device_id', '')
        side = table.get(i)
        if side is None and (what, i) not in self.unknown:
            self.unknown.add((what, i))
            self.get_logger().warn(f'{what} {i!r} not in config/haply.yaml: ignored')
        return side

    def _frame(self, j, t):
        for dev in j.get('inverse3') or []:
            side = self._side(self.inv_side, dev, 'inverse3')
            s = dev.get('state')
            if side is None or not s:
                continue
            frame = f'mtm_{side}_base'
            if 'cursor_position' in s:
                m = PointStamped()
                m.header.stamp, m.header.frame_id = t, frame
                p = s['cursor_position']
                m.point.x, m.point.y, m.point.z = float(p['x']), float(p['y']), float(p['z'])
                self.pub_pos[side].publish(m)
                tf = TransformStamped()
                tf.header.stamp, tf.header.frame_id = t, frame
                tf.child_frame_id = f'mtm_{side}_cursor'
                tf.transform.translation.x, tf.transform.translation.y = m.point.x, m.point.y
                tf.transform.translation.z = m.point.z
                tf.transform.rotation.w = 1.0
                q = self.orientation.get(side)
                if q is not None:
                    tf.transform.rotation = q
                self.tf.sendTransform(tf)
            if 'angular_position' in s:
                js = JointState()
                js.header.stamp = t
                js.name = [f'mtm_{side}_joint_{k + 1}' for k in range(3)]
                a = s['angular_position']
                js.position = [math.radians(float(a[f'a{k}'])) for k in range(3)]
                w = s.get('angular_velocity') or {}
                js.velocity = [math.radians(float(w.get(f'a{k}', 0.0))) for k in range(3)]
                self.pub_js[side].publish(js)
            if 'cursor_velocity' in s:
                m = Vector3Stamped()
                m.header.stamp, m.header.frame_id = t, frame
                v = s['cursor_velocity']
                m.vector.x, m.vector.y, m.vector.z = float(v['x']), float(v['y']), float(v['z'])
                self.pub_vel[side].publish(m)
        for dev in j.get('wireless_verse_grip') or []:
            side = self._side(self.grip_side, dev, 'versegrip')
            s = dev.get('state')
            if side is None or not s:
                continue
            key = (json.dumps(s.get('orientation'), sort_keys=True),
                   json.dumps(s.get('buttons'), sort_keys=True), s.get('hall'),
                   s.get('battery_level'))
            if self.last_grip.get(side) == key:
                continue
            self.last_grip[side] = key
            m = StylusState()
            m.header.stamp, m.header.frame_id = t, f'mtm_{side}_base'
            q = s.get('orientation')
            if q:
                m.orientation.w, m.orientation.x = float(q['w']), float(q['x'])
                m.orientation.y, m.orientation.z = float(q['y']), float(q['z'])
                self.orientation[side] = m.orientation
            b = s.get('buttons') or {}
            m.buttons = [bool(b.get(k, False)) for k in ('a', 'b', 'c')]
            m.grip = float(s.get('hall', 0))
            m.battery_level = float(s.get('battery_level', 0))
            self.pub_stylus[side].publish(m)

    def destroy_node(self):
        self.running = False
        if self.ws is not None:
            self.ws.close()
        self.thread.join(timeout=3.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = HaplyBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
