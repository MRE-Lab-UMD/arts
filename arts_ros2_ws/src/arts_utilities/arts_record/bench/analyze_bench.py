#!/usr/bin/env python3
"""Offline MCAP benchmark analyzer for ARTS modules.

Reads a rosbag2 MCAP recording and computes per-topic statistics:
  - Rate: mean +/- std from header.stamp deltas
  - Drop count: timestamp gaps > 1.5x nominal period
  - Total frames, duration, bag file size, disk throughput
  - Resource usage summary from a docker-stats CSV (optional)

Usage (inside a ROS 2 environment):
  python3 analyze_bench.py /path/to/bag_dir \\
      --nominal-hz 40.0 \\
      --docker-stats docker_stats.csv \\
      --image-digest sha256:abc123 \\
      --output-dir /bench_output
"""
import argparse
import csv
import json
import math
import os
import sys
from datetime import datetime, timezone
from pathlib import Path


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def _stdev(xs, mu=None):
    if len(xs) < 2:
        return 0.0
    if mu is None:
        mu = _mean(xs)
    return math.sqrt(sum((x - mu) ** 2 for x in xs) / (len(xs) - 1))


def analyze_bag(bag_path, nominal_hz):
    """Read MCAP bag and compute per-topic rate statistics."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image

    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(
        uri=str(bag_path), storage_id='mcap'
    )
    converter_opts = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_opts, converter_opts)

    topic_types = {
        t.name: t.type for t in reader.get_all_topics_and_types()
    }

    topic_stamps: dict[str, list[float]] = {}
    while reader.has_next():
        topic, data, _recv_ts = reader.read_next()
        if topic_types.get(topic) == 'sensor_msgs/msg/Image':
            msg = deserialize_message(data, Image)
            topic_stamps.setdefault(topic, []).append(
                _stamp_to_sec(msg.header.stamp)
            )

    nominal_period = 1.0 / nominal_hz
    drop_threshold = nominal_period * 1.5
    results: dict[str, dict] = {}

    for topic, stamps in topic_stamps.items():
        if len(stamps) < 2:
            continue
        stamps.sort()
        deltas = [stamps[i + 1] - stamps[i] for i in range(len(stamps) - 1)]
        rates = [1.0 / d for d in deltas if d > 0]
        drops = sum(1 for d in deltas if d > drop_threshold)

        rate_mu = _mean(rates)
        rate_sd = _stdev(rates, rate_mu)
        period_mu = _mean(deltas)
        period_sd = _stdev(deltas, period_mu)

        results[topic] = {
            'total_frames': len(stamps),
            'duration_s': round(stamps[-1] - stamps[0], 3),
            'rate_mean_hz': round(rate_mu, 2),
            'rate_std_hz': round(rate_sd, 2),
            'rate_min_hz': round(min(rates), 2),
            'rate_max_hz': round(max(rates), 2),
            'period_mean_ms': round(period_mu * 1000, 3),
            'period_std_ms': round(period_sd * 1000, 3),
            'drops': drops,
            'drop_threshold_ms': round(drop_threshold * 1000, 1),
            'nominal_hz': nominal_hz,
        }

    bag_p = Path(bag_path)
    if bag_p.is_file():
        total_bytes = bag_p.stat().st_size
    else:
        total_bytes = sum(
            f.stat().st_size for f in bag_p.rglob('*') if f.is_file()
        )

    return {
        'bag_path': str(bag_path),
        'bag_size_mb': round(total_bytes / 1e6, 2),
        'topics': results,
    }


def parse_docker_stats(csv_path):
    """Summarize docker-stats CSV (timestamp,cpu_pct,mem_usage,mem_pct)."""
    if not csv_path or not Path(csv_path).exists():
        return None
    cpu_vals = []
    mem_pct_vals = []
    mem_usage_strs = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                cpu_vals.append(float(row['cpu_pct'].strip('%')))
            except (ValueError, KeyError):
                continue
            try:
                mem_pct_vals.append(float(row['mem_pct'].strip('%')))
            except (ValueError, KeyError):
                pass
            mem_usage_strs.append(row.get('mem_usage', ''))

    if not cpu_vals:
        return None
    return {
        'cpu_mean_pct': round(_mean(cpu_vals), 1),
        'cpu_max_pct': round(max(cpu_vals), 1),
        'mem_mean_pct': round(_mean(mem_pct_vals), 1) if mem_pct_vals else None,
        'mem_max_pct': round(max(mem_pct_vals), 1) if mem_pct_vals else None,
        'mem_peak': mem_usage_strs[-1] if mem_usage_strs else None,
        'samples': len(cpu_vals),
    }


def format_markdown(report, image_digest, stats_summary=None, host_config=None):
    """Render a benchmark report as a markdown document."""
    lines = [
        '# FLIR Camera Benchmark Report',
        '',
        f'**Date:** {report.get("date", "N/A")}',
        f'**Camera:** FLIR Blackfly S BFS-U3-63S4C (USB3)',
        f'**Docker image digest:** `{image_digest}`',
        f'**Bag size:** {report["bag_size_mb"]} MB',
        '',
    ]

    # Results are only comparable across runs on a known host configuration.
    if host_config:
        isolated = host_config.get('isolated_cpus') or 'none'
        lines += [
            '## Host configuration',
            '',
            '| Setting | Value |',
            '|---------|-------|',
            f'| Kernel | {host_config.get("kernel_release", "?")} '
            f'({host_config.get("kernel_preempt", "?")}) |',
            f'| CPU | {host_config.get("cpu_model", "?")} '
            f'({host_config.get("nproc", "?")} logical) |',
            f'| Isolated CPUs | {isolated} |',
            f'| NVIDIA driver | {host_config.get("nvidia_driver", "?")} |',
            f'| net.core.rmem_max | {host_config.get("net_core_rmem_max", "?")} |',
            f'| usbfs_memory_mb | {host_config.get("usbfs_memory_mb", "?")} |',
            f'| Docker | {host_config.get("docker_version", "?")} |',
            '',
        ]
        if isolated == 'none':
            lines += [
                '> No CPU isolation was active for this run; it is not directly '
                'comparable with baselines taken under `isolcpus`.',
                '',
            ]

    for topic, s in report['topics'].items():
        duration_s = s['duration_s']
        disk_mbps = round(report['bag_size_mb'] / duration_s, 2) if duration_s > 0 else 0
        rate_pct = round(s['rate_mean_hz'] / s['nominal_hz'] * 100, 1)

        lines += [
            f'## `{topic}`',
            '',
            '| Metric | Value |',
            '|--------|-------|',
            f'| Nominal rate | {s["nominal_hz"]} Hz |',
            f'| Measured rate | {s["rate_mean_hz"]} &plusmn; {s["rate_std_hz"]} Hz ({rate_pct}% of nominal) |',
            f'| Rate range | [{s["rate_min_hz"]}, {s["rate_max_hz"]}] Hz |',
            f'| Period | {s["period_mean_ms"]} &plusmn; {s["period_std_ms"]} ms |',
            f'| Total frames | {s["total_frames"]} |',
            f'| Duration | {duration_s} s |',
            f'| Drops | {s["drops"]} (gap threshold: {s["drop_threshold_ms"]} ms) |',
            f'| Disk throughput | {disk_mbps} MB/s |',
            '',
        ]

    if stats_summary:
        lines += [
            '## Resource usage (container)',
            '',
            '| Metric | Value |',
            '|--------|-------|',
            f'| CPU mean | {stats_summary["cpu_mean_pct"]}% |',
            f'| CPU peak | {stats_summary["cpu_max_pct"]}% |',
        ]
        if stats_summary.get('mem_mean_pct') is not None:
            lines += [
                f'| Memory mean | {stats_summary["mem_mean_pct"]}% |',
                f'| Memory peak | {stats_summary["mem_max_pct"]}% ({stats_summary["mem_peak"]}) |',
            ]
        lines += [
            f'| Stat samples | {stats_summary["samples"]} (5 s interval) |',
            '',
        ]

    # Acceptance criteria
    lines += ['## Acceptance criteria', '']
    all_pass = True
    for topic, s in report['topics'].items():
        rate_pct = s['rate_mean_hz'] / s['nominal_hz'] * 100
        rate_ok = rate_pct >= 99.0
        drop_ok = s['drops'] == 0
        lines += [
            f'- **`{topic}`**',
            f'  - Rate >= 99% of nominal: **{"PASS" if rate_ok else "FAIL"}** ({rate_pct:.1f}%)',
            f'  - Zero drops: **{"PASS" if drop_ok else "FAIL"}** ({s["drops"]} drops)',
        ]
        if not (rate_ok and drop_ok):
            all_pass = False

    lines += ['', f'**Overall: {"PASS" if all_pass else "FAIL"}**', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description='ARTS MCAP benchmark analyzer')
    parser.add_argument('bag_path', help='Path to rosbag2 bag directory')
    parser.add_argument('--nominal-hz', type=float, default=40.0,
                        help='Expected publish rate (Hz)')
    parser.add_argument('--docker-stats', default='',
                        help='Path to docker_stats.csv from the run script')
    parser.add_argument('--image-digest', default='unknown',
                        help='Docker image digest string')
    parser.add_argument('--host-config', default='',
                        help='Path to host_config.json from the run script')
    parser.add_argument('--output-dir', default='.',
                        help='Directory for report files')
    args = parser.parse_args()

    bag_path = Path(args.bag_path)
    if not bag_path.exists():
        print(f'ERROR: bag path does not exist: {bag_path}', file=sys.stderr)
        sys.exit(1)
    # rosbag2 expects a directory with metadata.yaml.  If metadata.yaml is
    # missing (e.g. unclean shutdown), try the first .mcap file directly.
    if bag_path.is_dir() and not (bag_path / 'metadata.yaml').exists():
        mcap_files = sorted(bag_path.glob('*.mcap'))
        if mcap_files:
            print(f'[ANALYZE] WARNING: no metadata.yaml — reading '
                  f'{mcap_files[0].name} directly', flush=True)
            bag_path = mcap_files[0]

    print(f'[ANALYZE] Reading {bag_path} ...', flush=True)
    report = analyze_bag(bag_path, args.nominal_hz)
    report['date'] = datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC')

    stats = parse_docker_stats(args.docker_stats) if args.docker_stats else None

    host_config = None
    if args.host_config:
        host_path = Path(args.host_config)
        if host_path.exists():
            with open(host_path) as f:
                host_config = json.load(f)
        else:
            print(f'[ANALYZE] WARNING: host config not found: {host_path}',
                  file=sys.stderr)

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    json_path = out_dir / 'bench_report.json'
    report_out = {**report, 'image_digest': args.image_digest}
    if stats:
        report_out['resource_usage'] = stats
    if host_config:
        report_out['host_config'] = host_config
    with open(json_path, 'w') as f:
        json.dump(report_out, f, indent=2)
    print(f'[ANALYZE] JSON  -> {json_path}')

    md_path = out_dir / 'bench_report.md'
    md = format_markdown(report, args.image_digest, stats, host_config)
    with open(md_path, 'w') as f:
        f.write(md)
    print(f'[ANALYZE] MD    -> {md_path}')

    for topic, s in report['topics'].items():
        rate_pct = s['rate_mean_hz'] / s['nominal_hz'] * 100
        status = 'PASS' if (rate_pct >= 99 and s['drops'] == 0) else 'FAIL'
        print(f'[ANALYZE] {topic}: {s["rate_mean_hz"]} Hz '
              f'({rate_pct:.1f}%), {s["drops"]} drops -> {status}')


if __name__ == '__main__':
    main()
