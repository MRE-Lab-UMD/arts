#!/usr/bin/env python3
# Copyright 2026 ARTS contributors
# SPDX-License-Identifier: Apache-2.0
"""Measure collaborator A's effective homing-loop period on a real adapter bus. READ-ONLY.

A's tool_homing_node runs its state machine from a 10 ms timer, but every tick makes single
serial transactions: in the spin and limit phases, present position and present current of
the active disc, then present current and present position of all five discs for its
monitoring topics, 12 reads in all. If the callback takes longer than 10 ms, its duration IS
A's tick, and A's stall thresholds are tuned to it (engage_sequence.hpp, TIMING). This replays
that pattern and reports the period to set as disc_drive's engage_tick_period.

Only reads: no torque, mode or goal write. disc_drive must not be running on the same bus.
Python adds a little overhead per transaction relative to A's C++, so the result is a slight
over-estimate.

    source ~/arts/arts_ros2_ws/install/setup.bash
    python3 arts_ros2_ws/src/arts_embedded/arts_dv_tool_driver/test/a_tick_period_probe.py \
        --arm psm_left
"""
import argparse
import os
import statistics
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm', required=True, choices=['psm_left', 'psm_right'])
    ap.add_argument('--ticks', type=int, default=500)
    a = ap.parse_args()
    with open(os.path.join(get_package_share_directory('arts_dv_tool_driver'), 'config',
                           'disc_buses.yaml')) as f:
        port_name = yaml.safe_load(f)[a.arm]['port']
    with open(f'/sys/bus/usb-serial/devices/{os.path.basename(os.path.realpath(port_name))}'
              '/latency_timer') as f:
        latency = f.read().strip()
    port, pk = PortHandler(port_name), PacketHandler(2.0)
    if not (port.openPort() and port.setBaudRate(1000000)):
        raise SystemExit(f'cannot open {port_name}')
    errors = 0
    durations = []
    try:
        for _ in range(a.ticks):
            t0 = time.perf_counter()
            # A's tick in the spin phase, active disc 1: position, current; then all five.
            reads = [(1, ADDR_PRESENT_POSITION, 4), (1, ADDR_PRESENT_CURRENT, 2)]
            reads += [(i, ADDR_PRESENT_CURRENT, 2) for i in range(1, 6)]
            reads += [(i, ADDR_PRESENT_POSITION, 4) for i in range(1, 6)]
            for dxl_id, addr, n in reads:
                fn = pk.read4ByteTxRx if n == 4 else pk.read2ByteTxRx
                _, comm, err = fn(port, dxl_id, addr)
                errors += comm != COMM_SUCCESS or err != 0
            durations.append(time.perf_counter() - t0)
    finally:
        port.closePort()
    ms = [d * 1e3 for d in durations]
    q = statistics.quantiles(ms, n=20)
    print(f'{a.arm} {port_name}\nFTDI latency_timer {latency} ms, {a.ticks} ticks of 12 reads, '
          f'{errors} failed reads')
    print(f'tick duration mean {statistics.mean(ms):.2f} ms, std {statistics.stdev(ms):.2f}, '
          f'p5 {q[0]:.2f}, median {statistics.median(ms):.2f}, p95 {q[-1]:.2f}, '
          f'max {max(ms):.2f}')
    print(f'per read {statistics.mean(ms) / 12:.3f} ms. A\'s effective period = '
          f'max(10 ms timer, tick duration).')


if __name__ == '__main__':
    main()
