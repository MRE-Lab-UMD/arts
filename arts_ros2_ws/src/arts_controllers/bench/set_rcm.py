#!/usr/bin/env python3
"""Switch an arm's remote centre of motion on or off (<arm>/set_rcm), or show its state.

    set_rcm.py on                              # on the current shaft, at the last depth
    set_rcm.py on --depth 0.10                 # on the current shaft, 100 mm behind the tip
    set_rcm.py on --frame cannula_psm_left     # at a tf frame's origin (a calibrated cannula)
    set_rcm.py on --point 0.45 0.10 0.30       # a point in <arm>_base_link, metres
    set_rcm.py off
    set_rcm.py status                          # one /<arm>/rcm_state message

--arm (default $ARTS_ARM or psm_left). Plain "on" puts the point on the shaft where it is
NOW, at the depth of the last engaged RCM (rcm.default_insertion_depth, 0.10 m, before the
first), so off -> move the arm -> on relocates the RCM. --frame and --point name a fixed
point instead (a calibrated cannula); the controller refuses those unless the shaft already
passes within rcm.engage_max_deviation (2 mm) of it, so the arm never jumps onto a point.
The tip must be inserted within rcm.insertion_limits; the reply says which check failed. After either switch the controller holds until
a fresh target arrives, and the teleop follower disengages until the pedal is pressed again.
"""
import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from arts_interfaces.msg import RcmState
from arts_interfaces.srv import SetRcm


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('command', choices=['on', 'off', 'status'])
    ap.add_argument('--arm', default=os.environ.get('ARTS_ARM', 'psm_left'))
    where = ap.add_mutually_exclusive_group()
    where.add_argument('--depth', type=float, help='m behind the tip, on the current shaft')
    where.add_argument('--frame', help='tf frame whose origin is the RCM point')
    where.add_argument('--point', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                       help='m, in <arm>_base_link')
    args = ap.parse_args()
    base = f'{args.arm}_base_link'

    rclpy.init()
    node = Node('set_rcm')
    try:
        if args.command == 'status':
            got = []
            node.create_subscription(RcmState, f'/{args.arm}/rcm_state', got.append, 10)
            end = time.monotonic() + 3.0
            while not got and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not got:
                sys.exit(f'no /{args.arm}/rcm_state (is the servo controller active?)')
            m = got[0]
            p = m.point
            print(f'enabled {m.enabled}, constraint_ok {m.constraint_ok}; point '
                  f'[{p.x:.4f} {p.y:.4f} {p.z:.4f}] m in {m.header.frame_id}; deviation '
                  f'{m.deviation * 1e3:.4f} mm, insertion {m.insertion_depth * 1e3:.1f} mm')
            return 0

        req = SetRcm.Request(enable=args.command == 'on')
        if args.command == 'on' and args.frame is None and args.point is None:
            req.from_current_shaft = True
            req.insertion_depth = args.depth if args.depth is not None else 0.0  # 0: last depth
        elif args.point is not None:
            req.point.header.frame_id = base
            req.point.point.x, req.point.point.y, req.point.point.z = args.point
        elif args.frame is not None:
            # The controller does no tf lookups; transform the frame's origin into the base
            # frame here.
            buf = Buffer()
            TransformListener(buf, node)
            end = time.monotonic() + 3.0
            while not buf.can_transform(base, args.frame, Time()) and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            t = buf.lookup_transform(base, args.frame, Time()).transform.translation
            req.point.header.frame_id = base
            req.point.point.x, req.point.point.y, req.point.point.z = t.x, t.y, t.z
        client = node.create_client(SetRcm, f'/{args.arm}/set_rcm')
        if not client.wait_for_service(timeout_sec=5.0):
            sys.exit(f'/{args.arm}/set_rcm not available')
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        r = fut.result()
        if r is None:
            sys.exit('no response')
        print(('OK  ' if r.success else 'REFUSED  ') + r.message)
        return 0 if r.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
