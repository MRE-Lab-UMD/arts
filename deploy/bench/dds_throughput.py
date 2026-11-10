#!/usr/bin/env python3
"""Middleware throughput check: can the RMW carry the stereo laparoscope load?

    ./deploy/bench/dds_throughput.py                       # defaults below
    ./deploy/bench/dds_throughput.py --cameras 2 --seconds 30
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp ./deploy/bench/dds_throughput.py   # comparison

WHY THIS EXISTS SEPARATELY FROM THE FLIR BENCHMARK
--------------------------------------------------
The FLIR benchmark in arts_record/bench measures a module against real cameras and is the
number that gates acceptance. This measures the MIDDLEWARE alone, with synthetic frames of
the same size and cadence, and it exists because it answers questions the camera benchmark
cannot:

  - It runs with no hardware attached, so a middleware or kernel-buffer change can be
    evaluated the day it is made rather than the next time the rig is assembled.
  - It isolates transport from acquisition. A dropped frame in the camera benchmark could
    be USB, Spinnaker, the driver, the recorder or DDS; here it can only be DDS.
  - It runs identically on bare metal and inside a container, which is the comparison
    decision 4 requires and which was impossible until both paths ran the same RMW.

WHAT IT MEASURES AND WHAT IT DOES NOT
-------------------------------------
It reports, per camera topic, the delivered rate, the fraction of published frames that
arrived, and end-to-end latency from publish to callback. It also reports the SKEW between
the two topics' arrival times, because for stereo that is the number that matters: the
laparoscope pair is hardware-triggered, so any spread at the subscriber is transport
jitter, and it lands directly in the L/R stamp-skew budget in the evaluation table.

It does NOT measure what the real pipeline does with the frames -- no debayer, no encode,
no disk. It is a floor, not a prediction: if the middleware cannot carry the load with
nothing else running, it certainly cannot carry it with the recorder and the display
attached.

Publisher and subscriber are separate processes on purpose. In one process the RMW can
shortcut delivery and the numbers become meaningless.

DEFAULTS come from the hardware in the BOM: a Blackfly S BFS-U3-63S4C is 3072x2048, and
one 8-bit Bayer frame is 6.29 MB. Two of them at 40 Hz is 503 MB/s, which is the patient
cart's steady-state image load once the stereo laparoscope is running.
"""
import argparse
import json
import os
import statistics
import subprocess
import sys
import time

WIDTH, HEIGHT = 3072, 2048          # BFS-U3-63S4C
BYTES_PER_PIXEL = 1                 # 8-bit Bayer, as recorded (never RGB8; see AGENTS.md)
FRAME_BYTES = WIDTH * HEIGHT * BYTES_PER_PIXEL


def publisher_main(cameras, rate, seconds):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node("dds_bench_pub")

    # SENSOR-STYLE QoS, matched on both sides. Best-effort and depth 1 is what an image
    # pipeline should use: a late frame is worthless, so retransmitting it costs bandwidth
    # the next frame needs. Reliable QoS on a 6 MB sample is how an image topic takes the
    # whole domain down, and it is worth knowing that this benchmark deliberately does not
    # measure that case.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    pubs = [node.create_publisher(Image, f"/bench_dds/cam{i}/image_raw", qos)
            for i in range(cameras)]

    # One buffer, filled once. Allocating 6 MB per frame would measure the allocator.
    payload = bytes(FRAME_BYTES)
    templates = []
    for i in range(cameras):
        msg = Image()
        msg.height, msg.width = HEIGHT, WIDTH
        msg.encoding = "bayer_rggb8"
        msg.step = WIDTH * BYTES_PER_PIXEL
        msg.data = payload
        msg.header.frame_id = f"cam{i}"
        templates.append(msg)

    period = 1.0 / rate
    published = 0
    began = time.perf_counter()
    next_tick = began

    while time.perf_counter() - began < seconds:
        next_tick += period
        now = node.get_clock().now().to_msg()
        # Both cameras stamped identically and published back to back, standing in for the
        # shared hardware trigger. Any spread the subscriber sees is therefore transport.
        for msg, pub in zip(templates, pubs):
            msg.header.stamp = now
            pub.publish(msg)
        published += 1
        sleep_for = next_tick - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)

    # Let the last frames drain before the writer is torn down.
    time.sleep(0.5)
    print(json.dumps({"published": published}), flush=True)
    node.destroy_node()
    rclpy.shutdown()


def subscriber_main(cameras, seconds):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node("dds_bench_sub")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)

    # stamp -> {camera index: arrival time}, used for both latency and stereo skew.
    arrivals = {}
    counts = [0] * cameras
    latencies = [[] for _ in range(cameras)]

    def make_cb(index):
        def cb(msg):
            received = time.time()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            counts[index] += 1
            latencies[index].append(received - stamp)
            arrivals.setdefault(stamp, {})[index] = received
        return cb

    for i in range(cameras):
        node.create_subscription(Image, f"/bench_dds/cam{i}/image_raw", make_cb(i), qos)

    began = time.time()
    while time.time() - began < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)

    skews = [max(a.values()) - min(a.values())
             for a in arrivals.values() if len(a) == cameras]

    print(json.dumps({
        "counts": counts,
        "latency_ms": [[1000 * v for v in lat] for lat in latencies],
        "skew_ms": [1000 * s for s in skews],
        "complete_sets": len(skews),
    }), flush=True)
    node.destroy_node()
    rclpy.shutdown()


def summarize(values):
    if not values:
        return "n/a"
    values = sorted(values)
    mean = statistics.fmean(values)
    p99 = values[min(len(values) - 1, int(0.99 * len(values)))]
    return f"mean {mean:.2f} ms, p99 {p99:.2f} ms, max {values[-1]:.2f} ms"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cameras", type=int, default=2)
    ap.add_argument("--rate", type=float, default=40.0, help="Hz per camera")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--role", choices=["publisher", "subscriber"],
                    help="internal; the default run spawns both")
    args = ap.parse_args()

    if args.role == "publisher":
        return publisher_main(args.cameras, args.rate, args.seconds)
    if args.role == "subscriber":
        return subscriber_main(args.cameras, args.seconds)

    rmw = os.environ.get("RMW_IMPLEMENTATION", "<unset: RMW default>")
    uri = os.environ.get("CYCLONEDDS_URI", "<unset>")
    offered = args.cameras * args.rate * FRAME_BYTES / 1e6
    print(f"RMW_IMPLEMENTATION = {rmw}")
    print(f"CYCLONEDDS_URI     = {uri}")
    print(f"ROS_DOMAIN_ID      = {os.environ.get('ROS_DOMAIN_ID', '0')}")
    print(f"{args.cameras} x {WIDTH}x{HEIGHT} bayer_rggb8 "
          f"({FRAME_BYTES / 1e6:.2f} MB/frame) at {args.rate:g} Hz "
          f"= {offered:.0f} MB/s offered")
    print(f"running for {args.seconds:g} s ...\n")

    here = os.path.abspath(__file__)
    common = [sys.executable, here, "--cameras", str(args.cameras),
              "--rate", str(args.rate)]
    # The subscriber outlives the publisher so it can count the tail rather than cutting
    # the last second off and reporting it as loss.
    sub = subprocess.Popen(common + ["--seconds", str(args.seconds + 2), "--role",
                                     "subscriber"], stdout=subprocess.PIPE, text=True)
    time.sleep(2.0)  # discovery, before the first frame is sent
    pub = subprocess.Popen(common + ["--seconds", str(args.seconds), "--role",
                                     "publisher"], stdout=subprocess.PIPE, text=True)

    pub_out, _ = pub.communicate()
    sub_out, _ = sub.communicate()
    published = json.loads(pub_out.strip().splitlines()[-1])["published"]
    result = json.loads(sub_out.strip().splitlines()[-1])

    print(f"published   {published} frames per camera")
    worst_delivery = 1.0
    for i, count in enumerate(result["counts"]):
        delivered = count / published if published else 0.0
        worst_delivery = min(worst_delivery, delivered)
        rate = count / args.seconds
        print(f"cam{i}       {count} received ({100 * delivered:.1f}%), "
              f"{rate:.2f} Hz, {rate * FRAME_BYTES / 1e6:.0f} MB/s")
        print(f"            latency {summarize(result['latency_ms'][i])}")

    if args.cameras > 1:
        print(f"stereo skew {summarize(result['skew_ms'])} "
              f"over {result['complete_sets']} complete pairs")

    # 99% matches the recording-rate acceptance criterion in the evaluation table; the
    # middleware alone should clear it with room, since the real pipeline has to fit
    # debayer, encode and disk into the same budget.
    print()
    print("PASS" if worst_delivery >= 0.99
          else f"FAIL: worst delivery {100 * worst_delivery:.1f}% is below 99%")
    return 0 if worst_delivery >= 0.99 else 1


if __name__ == "__main__":
    sys.exit(main())
