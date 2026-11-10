#!/usr/bin/env python3
"""Disk-space guard for the image recorder.

The image bag fills a disk at ~94 MB/s (both cameras, 1536x1024, 30 Hz: 58.3 GB in 617.6 s,
docs/benchmarks/camera_composed_record_display_20261028.md). rosbag2 has no free-space limit
of its own, and a bag whose disk fills mid-write loses its tail and its metadata. So:

  pre-flight  --check-only --duration-s N: is there room for N seconds at the measured rate
              plus the reserve? Exit 0 yes, 2 no. Run it before every planned recording.
  live        (default) every --period-s, read free space on the bag's filesystem; below
              --reserve-gb call <recorder>/stop, which closes the bag cleanly, and exit 1.

The reserve is what the OS and everything else on the disk keep after a recording is stopped.
"""
import argparse
import os
import shutil
import sys
import time

GB = 1e9


def free_bytes(path):
    # The bag directory may not exist yet: measure the nearest existing parent.
    p = os.path.abspath(path)
    while not os.path.exists(p):
        p = os.path.dirname(p)
    return shutil.disk_usage(p).free


def preflight(args):
    free = free_bytes(args.path)
    need = args.rate_mbs * 1e6 * args.duration_s * args.margin
    left = free - need
    print('disk_guard: free %.1f GB; %.0f s at %.1f MB/s x %.2f = %.1f GB; '
          'after it %.1f GB, reserve %.1f GB'
          % (free / GB, args.duration_s, args.rate_mbs, args.margin, need / GB, left / GB,
             args.reserve_gb))
    if left < args.reserve_gb * GB:
        room_s = max(0.0, (free - args.reserve_gb * GB) / (args.rate_mbs * 1e6 * args.margin))
        print('disk_guard: NOT ENOUGH SPACE: room for %.1f min. Free space before recording.'
              % (room_s / 60), file=sys.stderr)
        return 2
    print('disk_guard: OK')
    return 0


def guard(args):
    import rclpy
    from rosbag2_interfaces.srv import Stop

    rclpy.init()
    node = rclpy.create_node('disk_guard')
    log = node.get_logger()
    client = node.create_client(Stop, args.recorder + '/stop')
    reserve = args.reserve_gb * GB
    last_log = 0.0
    try:
        while rclpy.ok():
            free = free_bytes(args.path)
            now = time.monotonic()
            if free < reserve:
                log.error('free %.1f GB < reserve %.1f GB: stopping %s'
                          % (free / GB, args.reserve_gb, args.recorder))
                if not client.wait_for_service(timeout_sec=5.0):
                    log.error('%s/stop not available; the recording was NOT stopped'
                              % args.recorder)
                    return 3
                future = client.call_async(Stop.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
                log.error('recorder stopped' if future.done() else 'stop call timed out')
                return 1
            if now - last_log >= args.log_period_s:
                log.info('free %.1f GB, %.1f min of recording left above the %.1f GB reserve'
                         % (free / GB, (free - reserve) / (args.rate_mbs * 1e6) / 60,
                            args.reserve_gb))
                last_log = now
            rclpy.spin_once(node, timeout_sec=args.period_s)
    except KeyboardInterrupt:  # launch shutdown sends SIGINT; that is a normal exit
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--path', required=True, help='bag path (or the directory it goes in)')
    ap.add_argument('--rate-mbs', type=float, default=94.4,
                    help='recording rate, MB/s (default: measured, both cameras binned)')
    ap.add_argument('--reserve-gb', type=float, default=30.0)
    ap.add_argument('--duration-s', type=float, default=0.0, help='planned length (pre-flight)')
    ap.add_argument('--margin', type=float, default=1.1, help='pre-flight size factor')
    ap.add_argument('--check-only', action='store_true')
    ap.add_argument('--recorder', default='/image_recorder')
    ap.add_argument('--period-s', type=float, default=1.0)
    ap.add_argument('--log-period-s', type=float, default=60.0)
    args, _ = ap.parse_known_args()  # tolerate --ros-args when started by launch
    sys.exit(preflight(args) if args.check_only else guard(args))


if __name__ == '__main__':
    main()
