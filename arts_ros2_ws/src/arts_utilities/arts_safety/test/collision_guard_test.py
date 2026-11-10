#!/usr/bin/env python3
"""collision_guard without arms: the geometry, and the hold / release / recovery logic on a
scripted tf scene.

    python3 collision_guard_test.py

Runs on its own ROS domain (ROS_DOMAIN_ID 77 unless set) so it never reaches a running cart.
Part A, geometry (no ROS): segment distances against hand cases, the capsule fit contains every
vertex of a real UR collision mesh. Part B: two fake "arms" (a robot_description with one
collision box each, written as an STL here) whose link frames this test moves on tf:
  1. apart: no hold;
  2. moved within 20 mm: hold, and the reason names the pair;
  3. release: recovery; moving closer by more than the slack: hold again;
  4. release, then apart beyond stop + clear margin: recovery ends, no hold;
  5. one arm's tf stops: hold (stale);
  6. a pedestal sphere between two frames 0.1 m apart: an arm link within 20 mm of it holds.
"""
import json
import os
import struct
import subprocess
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
import collision_guard as cg  # noqa: E402

FAILS = []


def check(ok, msg):
    print(f'{"PASS" if ok else "FAIL"} {msg}')
    if not ok:
        FAILS.append(msg)


def part_a():
    print('[A] geometry')
    P = lambda *v: np.array([v], float)  # noqa: E731
    d = cg.seg_seg_dist(P(0, 0, 0), P(1, 0, 0), P(0, 1, 0), P(1, 1, 0))[0]
    check(abs(d - 1.0) < 1e-9, f'parallel segments 1 m apart: {d:.6f}')
    d = cg.seg_seg_dist(P(0, 0, 0), P(1, 0, 0), P(0.5, -1, 0.3), P(0.5, 1, 0.3))[0]
    check(abs(d - 0.3) < 1e-9, f'crossing segments 0.3 m apart: {d:.6f}')
    d = cg.seg_seg_dist(P(0, 0, 0), P(1, 0, 0), P(2, 0, 0), P(3, 0, 0))[0]
    check(abs(d - 1.0) < 1e-9, f'collinear, end to end 1 m: {d:.6f}')
    d = cg.seg_seg_dist(P(0, 0, 0), P(0, 0, 0), P(1, 1, 0), P(1, 1, 0))[0]
    check(abs(d - np.sqrt(2)) < 1e-9, f'two points: {d:.6f}')
    from ament_index_python.packages import get_package_share_directory
    base = os.path.join(get_package_share_directory('ur_description'), 'meshes')
    for model in ('ur5e', 'ur5'):
        rs = []
        t0 = time.monotonic()
        for part in ('base', 'shoulder', 'upperarm', 'forearm', 'wrist1', 'wrist2', 'wrist3'):
            s = cg.surface_points(cg.read_stl(os.path.join(base, model, 'collision',
                                                           part + '.stl')))
            caps = cg.fit_capsules(s, k_max=1)   # as the guard: one capsule per UR link
            inside = np.zeros(len(s), bool)
            for a, b, r in caps:
                inside |= cg.seg_point_dist(a, b, s) <= r + 1e-9
            ext = s.max(axis=0) - s.min(axis=0)
            # No capsule may be fatter than the part's bounding box needs: its radius at most
            # half the box's largest diagonal (+ 5 mm).
            r_cap = 0.5 * np.sqrt(np.sum(np.sort(ext)[1:] ** 2)) + 0.005
            fat = [r for _, _, r in caps if r > max(r_cap, 0.5 * np.linalg.norm(ext) + 0.005)]
            rs.append(f'{part} {len(caps)}x r {",".join(str(round(1e3 * r)) for _, _, r in caps)}')
            if not inside.all():
                check(False, f'{model} {part}: {np.sum(~inside)} of {len(s)} SURFACE points outside')
            if fat:
                check(False, f'{model} {part}: capsule radius {fat} beyond the bounding box')
        check(True, f'{model}: capsules cover every surface point, none fatter than its box '
              f'({time.monotonic() - t0:.1f} s): ' + '; '.join(rs))


BOX = 0.05


def write_box_stl(path, half):
    v = np.array([[x, y, z] for x in (-half, half) for y in (-half, half) for z in (-half, half)])
    tris = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
            (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    with open(path, 'wb') as f:
        f.write(b'\0' * 80 + struct.pack('<I', len(tris)))
        for t in tris:
            f.write(struct.pack('<3f', 0, 0, 0))
            for k in t:
                f.write(struct.pack('<3f', *v[k]))
            f.write(b'\0\0')


def part_b():
    print('[B] hold logic on a scripted tf scene')
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from tf2_ros import TransformBroadcaster

    tmp = tempfile.mkdtemp(prefix='guard_test_')
    stl = os.path.join(tmp, 'box.stl')
    write_box_stl(stl, BOX)
    # A 100 mm box as the 'pedestal', 0.5 m from a1 along -x (no mounts in this scene).
    pedestal = os.path.join(tmp, 'pedestal.stl')
    write_box_stl(pedestal, 0.05)
    v = cg.read_stl(pedestal) + np.array([-0.5, 0.0, 0.0])
    with open(pedestal, 'wb') as f:
        f.write(b'\0' * 80 + struct.pack('<I', len(v) // 3))
        for t in range(len(v) // 3):
            f.write(struct.pack('<3f', 0, 0, 0))
            for q in v[3 * t:3 * t + 3]:
                f.write(struct.pack('<3f', *q))
            f.write(b'\0\0')
    urdf = lambda arm: (  # noqa: E731
        f'<robot name="{arm}"><link name="{arm}_upper_arm_link"><collision><geometry>'
        f'<mesh filename="file://{stl}"/></geometry></collision></link></robot>')
    cfg = os.path.join(tmp, 'guard.yaml')
    with open(cfg, 'w') as f:
        f.write('/collision_guard:\n  ros__parameters:\n    arms: [a1, a2]\n'
                '    world_frame: world\n    rate: 50.0\n    stop_distance: 0.02\n'
                '    warn_distance: 0.05\n    clear_margin: 0.02\n    recovery_slack: 0.002\n'
                '    stale_s: 0.3\n    description_wait_s: 2.0\n'
                '    pedestal_spheres: \'[{"name": "holder", "between": ["m1", "m2"], '
                '"diameter_scale": 1.0}]\'\n'
                f'    cache_dir: {os.path.join(tmp, "cache")}\n')
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID', '77'))
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    guard = subprocess.Popen([sys.executable, os.path.join(HERE, '..', 'scripts',
                                                           'collision_guard.py'),
                              '--ros-args', '--params-file', cfg], env=env)
    rclpy.init()
    n = rclpy.create_node('guard_test')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for arm in ('a1', 'a2'):
        n.create_publisher(String, f'/{arm}/robot_description', latched).publish(
            String(data=urdf(arm)))
    pubs = n._publishers[-2:]  # keep the latched publishers alive  # noqa: F841
    tfb = TransformBroadcaster(n)
    st = {'hold': None, 'state': None}
    n.create_subscription(Bool, '/arts/collision_hold', lambda m: st.__setitem__('hold', m.data),
                          latched)
    n.create_subscription(String, '/collision_guard/state',
                          lambda m: st.__setitem__('state', json.loads(m.data)), 10)
    rel = n.create_client(Trigger, '/collision_guard/release')
    x2 = {'v': 0.5}
    x1 = {'v': 0.0}
    live = {'a2': True}

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            now = n.get_clock().now().to_msg()
            for frame, y in (('m1', -0.05), ('m2', 0.05)):
                t = TransformStamped()
                t.header.stamp, t.header.frame_id = now, 'world'
                t.child_frame_id = frame
                t.transform.translation.x, t.transform.translation.y = -0.5, y
                t.transform.rotation.w = 1.0
                tfb.sendTransform(t)
            for arm, x in (('a1', x1['v']), ('a2', x2['v'])):
                if arm == 'a2' and not live['a2']:
                    continue
                t = TransformStamped()
                t.header.stamp, t.header.frame_id = now, 'world'
                t.child_frame_id = f'{arm}_upper_arm_link'
                t.transform.translation.x = x
                t.transform.rotation.w = 1.0
                tfb.sendTransform(t)
            rclpy.spin_once(n, timeout_sec=0.01)

    def release():
        fut = rel.call_async(Trigger.Request())
        end = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < end:
            spin(0.05)
        return fut.result()

    try:
        rel.wait_for_service(timeout_sec=10.0)
        spin(4.0)
        # Box half 50 mm: the fitted capsule radius is the corner distance from its axis.
        closest = st['state']['closest'][0]['mm'] if st['state'] else None
        check(st['hold'] is False, f'[1] apart: no hold (closest {closest} mm)')
        surface0 = closest / 1e3 if closest is not None else None
        # Place a2 so the surface distance is 10 mm.
        x2['v'] = 0.5 - surface0 + 0.010
        spin(1.0)
        reason = st['state']['reason'] if st['state'] else ''
        check(st['hold'] is True and 'a1/upper_arm_link' in reason and 'a2/upper_arm_link' in reason,
              f'[2] within 20 mm: hold ({reason})')
        r = release()
        check(r is not None and r.success and 'recovery' in r.message,
              f'[3] release: {r.message if r else "no response"}')
        spin(0.5)
        check(st['hold'] is False, '[3] released: no hold while still at 10 mm')
        x2['v'] -= 0.004
        spin(0.8)
        check(st['hold'] is True, f'[3] 4 mm closer during recovery: hold again '
              f'({st["state"]["reason"] if st["state"] else ""})')
        release()
        x2['v'] += 0.060
        spin(1.0)
        check(st['hold'] is False and st['state'] and not st['state']['recovery'],
              '[4] separated beyond stop + margin: recovery ended, no hold')
        live['a2'] = False
        spin(1.0)
        check(st['hold'] is True and 'old' in st['state']['reason'] if st['state'] else False,
              f'[5] a2 tf stopped: hold ({st["state"]["reason"] if st["state"] else ""})')
        live['a2'] = True
        spin(1.0)
        x1['v'] = -0.5 + 0.05 + 0.050 + 0.010        # sphere radius + a1's capsule x-reach + 10 mm
        spin(1.0)
        reason = st['state']['reason'] if st['state'] else ''
        check(st['hold'] is True and 'pedestal/holder' in reason,
              f'[6] a1 near the pedestal sphere: hold ({reason})')
    finally:
        guard.terminate()
        guard.wait(timeout=5)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    part_a()
    part_b()
    print('\nPASS' if not FAILS else f'\n{len(FAILS)} FAILED')
    sys.stdout.flush()
    os._exit(1 if FAILS else 0)
