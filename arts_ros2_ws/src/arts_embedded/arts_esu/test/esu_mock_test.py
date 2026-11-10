#!/usr/bin/env python3
"""The esu node against the fake ESU-TAP board (decision 18, 2026-11-03 amendment).

    source arts_ros2_ws/install/setup.bash
    python3 arts_ros2_ws/src/arts_embedded/arts_esu/test/esu_mock_test.py

The fake board and the esu node run on rmw_fastrtps_cpp, as on the cart; this rig runs on the
system default (CycloneDDS), as the rest of ARTS does, so every check also crosses DDS vendors.
Board hooks are set with `ros2 param set` on Fast DDS. Checks:

  1. the node refuses to start on CycloneDDS (the services would never answer)
  2. /esu/info: board v2, three channels, monopolar -> psm_left, bipolar -> psm_right (tool_info)
  3. a target before enable is refused: nothing switches
  4. enable, then a held psm_left CUT target: monopolar CUT on, source software, routed to
     psm_left; the 1 Hz snapshots (source sync) carry it; /esu/command ACTIVATE, 0.5 s expiry
  5. targets stop: CUT off within target_timeout + a renewal; DEACTIVATE published
  6. psm_right: bipolar on; activate=false switches it off at once
  7. BLEND refused
  8. a bipolar pedal pressed while software CUT is on: CUT released as a takeover, bipolar on
     from the pedal, software disabled; the still-streaming CUT target stays refused
  9. enable while the pedal is held is refused
 10. both arms monopolar: routing refused, a psm_left target is refused
 11. a pedal press and release delivered together are flagged timing_suspect; a normal
     0.3 s press is not (the detector's failure path)
 12. the node killed (SIGKILL) with software CUT on: the board's lease switches it off
"""
import os
import signal
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32

from arts_interfaces.msg import EnergyCommand, EnergyState, EnergyTarget, EsuInfo, ToolInfo

FAILS = []
FAST = dict(os.environ, RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def check(name, ok, detail=''):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}{': ' + detail if detail else ''}", flush=True)
    if not ok:
        FAILS.append(name)


def board_param(rig, name, value, target=None):
    """Set a fake-board hook; the rig keeps spinning (and sending target) meanwhile."""
    p = subprocess.Popen(['ros2', 'param', 'set', '/fake_esu_tap', name, value], env=FAST,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    while p.poll() is None:
        rig.spin(0.02, target)
    if p.returncode != 0:
        raise RuntimeError(f'ros2 param set {name} {value} failed')


class Rig(Node):

    def __init__(self):
        super().__init__('esu_mock_test')
        self.states, self.cmds, self.info = [], [], None
        self.create_subscription(EnergyState, '/esu/state', self.states.append, 100)
        self.create_subscription(EnergyCommand, '/esu/command', self.cmds.append, 100)
        self.create_subscription(EsuInfo, '/esu/info', lambda m: setattr(self, 'info', m), LATCHED)
        self.target = self.create_publisher(EnergyTarget, '/esu/target', 10)
        self.enable = self.create_publisher(Bool, '/esu/enable', 10)
        self.tool = {a: self.create_publisher(ToolInfo, f'/{a}/tool_info', LATCHED)
                     for a in ('psm_left', 'psm_right')}

    def spin(self, s, target=None):
        end = time.monotonic() + s
        nxt = 0.0
        while time.monotonic() < end:
            if target is not None and time.monotonic() >= nxt:
                self.target.publish(target)
                nxt = time.monotonic() + 0.05
            rclpy.spin_once(self, timeout_sec=0.005)

    def last(self):
        return self.states[-1] if self.states else None

    def set_tool(self, arm, energy):
        self.tool[arm].publish(ToolInfo(name='test', energy_type=energy))


def tgt(arm='', output=0, mode=EnergyCommand.CUT, activate=True):
    return EnergyTarget(arm=arm, output=output, mode=mode, activate=activate)


def start(script, env):
    """The installed script itself, not `ros2 run` (a wrapper whose death orphans the node)."""
    path = os.path.join(get_package_prefix('arts_esu'), 'lib', 'arts_esu', script)
    return subprocess.Popen([path], env=env, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)


def stop(p):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()


def main():
    print('[1] RMW guard')
    bad = subprocess.run(['ros2', 'run', 'arts_esu', 'esu_node.py'], capture_output=True,
                         text=True, timeout=20,
                         env=dict(os.environ, RMW_IMPLEMENTATION='rmw_cyclonedds_cpp'))
    check('refuses CycloneDDS', bad.returncode != 0 and 'rmw_fastrtps_cpp' in bad.stderr + bad.stdout,
          (bad.stderr + bad.stdout).strip().splitlines()[-1][:120] if (bad.stderr + bad.stdout) else '')

    fake = start('fake_esu_tap.py', FAST)
    node = start('esu_node.py', FAST)
    rclpy.init()
    rig = Rig()
    try:
        rig.set_tool('psm_left', ToolInfo.ENERGY_MONOPOLAR)
        rig.set_tool('psm_right', ToolInfo.ENERGY_BIPOLAR)
        rig.spin(5.0)

        print('[2] info and routing')
        i = rig.info
        check('info', i is not None and i.board == 'v2' and list(i.channels) ==
              ['cut', 'coag', 'bipolar'] and dict(zip(i.outputs, i.arms)) ==
              {EnergyCommand.MONOPOLAR: 'psm_left', EnergyCommand.BIPOLAR: 'psm_right'},
              f'{i.board} {list(i.channels)} {list(zip(i.outputs, i.arms, i.mapping_source))}'
              if i else 'no info')

        print('[3] target before enable')
        n0 = len(rig.states)
        rig.spin(0.8, tgt('psm_left'))
        check('refused, nothing switched', not any(s.monopolar_cut for s in rig.states[n0:])
              and not rig.cmds, f'{len(rig.cmds)} commands')

        print('[4] enable + held psm_left CUT')
        rig.enable.publish(Bool(data=True))
        rig.spin(0.5)
        check('enabled', rig.last() is not None and rig.last().software_enabled)
        n0 = len(rig.states)
        rig.spin(1.0, tgt('psm_left'))
        ev = [x for x in rig.states[n0:] if x.source != EnergyState.SOURCE_SYNC]  # not snapshots
        s = ev[0] if ev else rig.last()
        check('CUT on from software, routed to psm_left',
              s.monopolar_cut and s.source == EnergyState.SOURCE_SOFTWARE and
              list(s.active_arms) == ['psm_left'] and rig.last().monopolar_cut,
              f'{s.monopolar_cut} src {s.source} {list(s.active_arms)}')
        snaps = [x for x in rig.states[n0:] if x.source == EnergyState.SOURCE_SYNC]
        check('1 Hz snapshots carry the current state', len(snaps) >= 1 and
              all(x.monopolar_cut for x in snaps[1:]), f'{len(snaps)} in 1 s')
        c = rig.cmds[0] if rig.cmds else None
        check('command ACTIVATE, expiry 0.5 s', c is not None and c.action == c.ACTIVATE and
              c.arm == 'psm_left' and c.expiry.nanosec == 500000000)

        print('[5] targets stop')
        t0 = time.monotonic()
        while rig.last().monopolar_cut and time.monotonic() - t0 < 2.0:
            rig.spin(0.01)
        dt = time.monotonic() - t0
        check('CUT off after the target went stale', not rig.last().monopolar_cut and dt < 0.6,
              f'{dt * 1e3:.0f} ms')
        check('DEACTIVATE published', rig.cmds[-1].action == EnergyCommand.DEACTIVATE)

        print('[6] psm_right bipolar, then activate=false')
        rig.spin(0.6, tgt('psm_right'))
        check('bipolar on, psm_right', rig.last().bipolar and
              list(rig.last().active_arms) == ['psm_right'])
        t0 = time.monotonic()
        rig.target.publish(tgt('psm_right', activate=False))
        while rig.last().bipolar and time.monotonic() - t0 < 1.0:
            rig.spin(0.005)
        check('off at once', not rig.last().bipolar and time.monotonic() - t0 < 0.15,
              f'{(time.monotonic() - t0) * 1e3:.0f} ms')

        print('[7] BLEND')
        n0 = len(rig.cmds)
        rig.spin(0.5, tgt(output=EnergyCommand.MONOPOLAR, mode=EnergyCommand.BLEND))
        check('refused', len(rig.cmds) == n0 and not rig.last().monopolar_cut)

        print('[8] pedal takeover')
        rig.spin(0.6, tgt('psm_left'))
        on_before = rig.last().monopolar_cut
        n0 = len(rig.states)
        board_param(rig, 'pedals', "['bipolar']", tgt('psm_left'))
        rig.spin(1.0, tgt('psm_left'))
        seq = [(x.monopolar_cut, x.bipolar, x.source) for x in rig.states[n0:]]
        check('CUT released as takeover, bipolar on from the pedal',
              on_before and (False, False, EnergyState.SOURCE_TAKEOVER) in seq and
              (False, True, EnergyState.SOURCE_FOOTSWITCH) in seq, f'{seq}')
        check('software disabled, CUT target stays refused',
              not rig.last().software_enabled and not rig.last().monopolar_cut)

        print('[9] enable with a pedal held')
        rig.enable.publish(Bool(data=True))
        rig.spin(0.5)
        check('refused', not rig.last().software_enabled)
        board_param(rig, 'pedals', "['']")
        rig.spin(0.3)

        print('[10] both arms monopolar')
        rig.set_tool('psm_right', ToolInfo.ENERGY_MONOPOLAR)
        rig.spin(0.5)
        rig.enable.publish(Bool(data=True))
        rig.spin(0.5)
        n0 = len(rig.cmds)
        rig.spin(0.6, tgt('psm_left'))
        check('routing refused', len(rig.cmds) == n0 and not rig.last().monopolar_cut and
              EnergyCommand.MONOPOLAR not in list(rig.info.outputs),
              f'{list(zip(rig.info.outputs, rig.info.arms))}')
        rig.set_tool('psm_right', ToolInfo.ENERGY_BIPOLAR)
        rig.spin(0.5)

        print('[11] delivery held back')
        n0 = len(rig.states)
        board_param(rig, 'pedals', "['cut']")
        rig.spin(0.3)
        board_param(rig, 'pedals', "['']")
        rig.spin(0.5)
        normal = [x.timing_suspect for x in rig.states[n0:]]
        check('a normal press is not flagged', len(normal) >= 2 and not any(normal), f'{normal}')
        n0 = len(rig.states)
        board_param(rig, 'hold_delivery', 'true')
        board_param(rig, 'pedals', "['coag']")
        rig.spin(0.2)
        board_param(rig, 'pedals', "['']")
        rig.spin(0.2)
        board_param(rig, 'hold_delivery', 'false')
        rig.spin(0.5)
        flagged = [(x.monopolar_coag, x.timing_suspect) for x in rig.states[n0:]]
        check('press + release together flagged', (False, True) in flagged, f'{flagged}')

        print('[12] node killed with software CUT on')
        rig.enable.publish(Bool(data=True))
        rig.spin(0.5)
        rig.spin(0.6, tgt('psm_left'))
        on = rig.last().monopolar_cut
        sub_raw = []  # with the node dead, hear the board itself
        rig.create_subscription(Int32, '/esu/activation_state', sub_raw.append, 10)
        rig.spin(0.3, tgt('psm_left'))
        sub_raw.clear()
        t0 = time.monotonic()
        node.send_signal(signal.SIGKILL)
        node.wait(timeout=2)
        dead = not os.path.exists(f'/proc/{node.pid}')
        # keep asking for CUT: with the node dead, only the board's lease can end it
        while not sub_raw and time.monotonic() - t0 < 2.0:
            rig.spin(0.01, tgt('psm_left'))
        dt = time.monotonic() - t0
        check('node really killed', dead, f'pid {node.pid}')
        check('lease switched CUT off', on and sub_raw and (sub_raw[-1].data & 1) == 0 and
              0.35 < dt < 0.7, f'{dt * 1e3:.0f} ms after the kill (lease 500 ms after the last '
              'renewal, renewals every 100 ms)')
    finally:
        rig.destroy_node()
        rclpy.shutdown()
        stop(node)
        stop(fake)
    print('\nPASS' if not FAILS else f'\n{len(FAILS)} FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
