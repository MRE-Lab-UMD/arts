#!/usr/bin/env python3
"""esu: ARTS's side of the ESU-TAP board (decision 18, 2026-11-03 amendment).

The board runs the ESU-TAP designer's firmware unmodified (protocol 2, pinned in
oem_drivers/esu_tap/VERSION). It sits in the footswitch cables between the pedals and the ESU:
its pedals act on the ESU directly, without ROS, and software can close the same contacts. Its
micro-ROS session is bridged by micro_ros_agent and offers, under /esu:

  activation_state  std_msgs/Int32     bits 0-3 cut, coag, bipolar, ligasure; bits 4-5 source
  cut coag bipolar  std_srvs/SetBool   switch one channel (ON expires lease_ms after its last
                                       renewal, so it is renewed every refresh_ms)
  arm               std_srvs/SetBool   accept software commands ("enable" here; any debounced
                                       pedal press clears it, and it must be set again deliberately)
  capabilities      std_srvs/Trigger   JSON: board, firmware, channels, lease, boot id, ...

This node turns that into the decision-18 contract. The logic is the designer's PC client
(pedal_controller: capability check, lease renewal, missed-renewal disarm, logger edges),
re-derived for ARTS, not copied.

RMW: the agent is built on Fast DDS, and Fast DDS and CycloneDDS do not interoperate on
services (topics do). This node therefore runs on rmw_fastrtps_cpp (esu.launch.py sets it) and
refuses to start on anything else. For the same reason everything it offers to the rest of ARTS
is a topic, never a service.

Topics out (/esu): info (EsuInfo, latched, on every (re)connection and mapping change),
device_state and state (EnergyState, on every board message, plus a 1 Hz snapshot with source
SYNC so a late subscriber sees the state and its liveness; state adds the arms), command
(EnergyCommand, when a channel is switched). The board's own activation_state stays on the graph
and is recorded raw.
Topics in: /esu/target (EnergyTarget, hold-to-run), /esu/enable (std_msgs/Bool: each message
is one deliberate act; true asks the board to accept software commands, false switches software
energy off and disables), /psm_left/tool_info, /psm_right/tool_info.

Routing: the monopolar output feeds the one arm whose instrument is monopolar, the bipolar
output the one arm whose instrument is bipolar (tool_info); monopolar_arm / bipolar_arm
override. An ambiguous or missing mapping refuses targets addressed by arm.

Software energy: one output at a time; a target older than target_timeout_s switches it off;
two renewals in a row unanswered disable software control (the designer's rule). The board's
lease is the authority if this node dies.

Timing: the stamps are this node's receipt times (on Fast DDS, on time; the board stamps
nothing), so energy timing comes from /esu/state and device_state, not from the raw topic,
which a CycloneDDS recorder that joined late can log up to 3 s late (decision 18, 2026-11-03).
timing_suspect guards the node's own input: the firmware ignores pedal changes shorter than its
30 ms debounce, so a press and its release arriving closer than that were delayed upstream; the
release is flagged and a warning logged (never seen at the node).

Parameters: esu_model (DRE ASG-300), monopolar_arm, bipolar_arm (empty = from tool_info),
target_timeout_s (0.3), debounce_s (0.030), call_timeout_s (0.2).
"""
import json

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32
from std_srvs.srv import SetBool, Trigger

from arts_interfaces.msg import EnergyCommand, EnergyState, EnergyTarget, EsuInfo, ToolInfo

CHANNELS = ('cut', 'coag', 'bipolar', 'ligasure')
REVISION_CHANNELS = {'v1': 2, 'v2': 3, 'v3': 4}
ARMS = ('psm_left', 'psm_right')
MONO, BI = EnergyCommand.MONOPOLAR, EnergyCommand.BIPOLAR
OUTPUT_NAME = {MONO: 'monopolar', BI: 'bipolar'}
TOOL_ENERGY = {MONO: ToolInfo.ENERGY_MONOPOLAR, BI: ToolInfo.ENERGY_BIPOLAR}
CHANNEL_OUTPUT = {'cut': MONO, 'coag': MONO, 'bipolar': BI}


def parse_capabilities(text):
    """The board's capability JSON, checked as strictly as the designer's client does."""
    d = json.loads(text)
    if not isinstance(d, dict) or d.get('protocol') != 2:
        raise ValueError('not protocol-2 firmware')
    if d.get('board') not in REVISION_CHANNELS:
        raise ValueError(f'unknown board {d.get("board")!r}')
    if d.get('channel_count') != REVISION_CHANNELS[d['board']]:
        raise ValueError('channel count does not match the board revision')
    if d.get('requires_arm') is not True or not isinstance(d.get('armed'), bool):
        raise ValueError('unsupported control policy')
    for k in ('lease_ms', 'refresh_ms', 'boot_id', 'dropped_events'):
        if type(d.get(k)) is not int or d[k] < 0:
            raise ValueError(f'bad {k}')
    if not 0 < d['refresh_ms'] < d['lease_ms'] // 2:
        raise ValueError('refresh must be under half the lease')
    d['channels'] = list(CHANNELS[:d['channel_count']])
    return d


class Esu(Node):

    def __init__(self):
        super().__init__('esu')
        rmw = rclpy.get_rmw_implementation_identifier()
        if rmw != 'rmw_fastrtps_cpp':
            raise SystemExit(f'esu needs RMW_IMPLEMENTATION=rmw_fastrtps_cpp (running {rmw}): the '
                             'micro-ROS agent is Fast DDS and services do not cross to CycloneDDS')
        p = self.declare_parameter
        self.esu_model = p('esu_model', 'DRE ASG-300').value
        self.override = {MONO: p('monopolar_arm', '').value, BI: p('bipolar_arm', '').value}
        self.target_timeout = p('target_timeout_s', 0.3).value
        self.debounce = p('debounce_s', 0.030).value
        self.call_timeout = p('call_timeout_s', 0.2).value

        self.caps = None              # parsed capabilities of the connected board
        self.caps_future = None
        self.connected = False
        self.enabled = False
        self.bits = None              # last board bits
        self.press_t = {}             # channel -> receipt time of its last pedal press
        self.tool = dict.fromkeys(ARMS)
        self.mapping = {}             # output -> (arm, source)
        self.want = None              # dict: channel, output, mode, arm, source, t (last target)
        self.on = None                # channel software holds ON at the board
        self.pending = None           # (future, sent_t, kind)
        self.missed = 0
        self.last_renew = 0.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_info = self.create_publisher(EsuInfo, '/esu/info', latched)
        self.pub_dev = self.create_publisher(EnergyState, '/esu/device_state', 100)
        self.pub_state = self.create_publisher(EnergyState, '/esu/state', 100)
        self.pub_cmd = self.create_publisher(EnergyCommand, '/esu/command', 100)
        self.create_subscription(Int32, '/esu/activation_state', self._on_board, 100)
        self.create_subscription(EnergyTarget, '/esu/target', self._on_target, 10)
        self.create_subscription(Bool, '/esu/enable', self._on_enable, 10)
        for arm in ARMS:
            self.create_subscription(ToolInfo, f'/{arm}/tool_info',
                                     lambda m, a=arm: self._on_tool(a, m), latched)
        self.cli_caps = self.create_client(Trigger, '/esu/capabilities')
        self.cli_arm = self.create_client(SetBool, '/esu/arm')
        self.cli = {ch: self.create_client(SetBool, f'/esu/{ch}') for ch in CHANNEL_OUTPUT}
        self.create_timer(1.0, self._link)
        self.create_timer(0.02, self._tick)
        self.get_logger().info('esu: waiting for the ESU-TAP board (is micro_ros_agent running?)')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- board link and capabilities ----
    def _link(self):
        up = (self.cli_caps.service_is_ready() and
              self.count_publishers('/esu/activation_state') == 1)
        if self.count_publishers('/esu/activation_state') > 1:
            self.get_logger().error('more than one publisher on /esu/activation_state: one board '
                                    '(or fake) per ROS domain')
        if up and not self.connected and self.caps_future is None:
            self.caps_future = self.cli_caps.call_async(Trigger.Request())
            self.caps_future.add_done_callback(self._on_caps)
        elif not up and self.connected:
            self.get_logger().warn('board disconnected: software energy disabled; pedals still '
                                   'work on the board')
            self.connected = self.enabled = False
            self._drop_software('board disconnected')
            self._publish_state(self.bits or 0, EnergyState.SOURCE_SYNC, False)
        else:
            self._publish_state(self.bits or 0, EnergyState.SOURCE_SYNC, False)  # 1 Hz snapshot

    def _on_caps(self, fut):
        self.caps_future = None
        try:
            r = fut.result()
            if not r.success:
                raise ValueError('capabilities call failed')
            caps = parse_capabilities(r.message)
        except Exception as e:  # noqa: BLE001 (the board's reply is untrusted input)
            self.get_logger().error(f'board capabilities rejected: {e}')
            return
        if self.caps and caps['boot_id'] != self.caps['boot_id']:
            self.get_logger().warn(f'board rebooted (boot id {self.caps["boot_id"]} -> '
                                   f'{caps["boot_id"]})')
        if self.caps and caps['dropped_events'] != self.caps['dropped_events']:
            self.get_logger().warn(f'board dropped {caps["dropped_events"] - self.caps["dropped_events"]}'
                                   ' state messages: the recording has a gap')
        self.caps, self.connected, self.enabled = caps, True, caps['armed']
        self.get_logger().info(
            f'board {caps["board"]}, firmware {caps["firmware"]}, channels {caps["channels"]}, '
            f'pedals {"read" if caps["pedal_connected"] else "IGNORED"}, lease {caps["lease_ms"]} '
            f'ms, boot {caps["boot_id"]}, software {"enabled" if self.enabled else "disabled"}')
        self._remap()
        self._publish_info()

    def _publish_info(self):
        c = self.caps or {}
        m = EsuInfo(esu_model=self.esu_model, board=c.get('board', ''),
                    board_firmware=c.get('firmware', ''), channels=c.get('channels', []),
                    pedal_connected=c.get('pedal_connected', False),
                    lease_ms=c.get('lease_ms', 0), boot_id=c.get('boot_id', 0),
                    dropped_events=c.get('dropped_events', 0))
        for out, (arm, src) in sorted(self.mapping.items()):
            m.outputs.append(out)
            m.arms.append(arm)
            m.mapping_source.append(src)
        m.header.stamp = self.get_clock().now().to_msg()
        self.pub_info.publish(m)

    # ---- routing ----
    def _on_tool(self, arm, m):
        self.tool[arm] = m.energy_type
        self._remap()
        self._publish_info()

    def _remap(self):
        mapping = {}
        for out in (MONO, BI):
            if self.override[out]:
                mapping[out] = (self.override[out], 'config')
                continue
            arms = [a for a in ARMS if self.tool[a] == TOOL_ENERGY[out]]
            if len(arms) == 1:
                mapping[out] = (arms[0], 'tool_info')
            elif len(arms) > 1:
                self.get_logger().warn(f'{OUTPUT_NAME[out]}: both arms carry a '
                                       f'{OUTPUT_NAME[out]} instrument; not routed (set '
                                       f'{OUTPUT_NAME[out]}_arm)')
        if self.on and mapping.get(self.want['output']) != self.mapping.get(self.want['output']):
            self._drop_software('the instrument routing changed while energy was on')
        if mapping != self.mapping:
            self.get_logger().info('routing: ' + (', '.join(
                f'{OUTPUT_NAME[o]} -> {a} ({s})' for o, (a, s) in sorted(mapping.items()))
                or 'none'))
        self.mapping = mapping

    # ---- board state ----
    def _on_board(self, m):
        now = self._now()
        bits, src = m.data & 0xF, (m.data >> 4) & 0x3
        suspect = False
        if self.caps:
            for i, ch in enumerate(self.caps['channels']):
                was = self.bits is not None and bool(self.bits >> i & 1)
                on = bool(bits >> i & 1)
                if on == was or self.bits is None:
                    continue
                if src == EnergyState.SOURCE_FOOTSWITCH and on:
                    self.press_t[ch] = now
                elif src == EnergyState.SOURCE_FOOTSWITCH and ch in self.press_t:
                    held = now - self.press_t.pop(ch)
                    if held < self.debounce:
                        suspect = True
                        self.get_logger().warn(
                            f'{ch}: pedal press and release arrived {held * 1e3:.1f} ms apart, '
                            f'under the board\'s {self.debounce * 1e3:.0f} ms debounce: delivery '
                            'was delayed upstream, the press time is late (timing_suspect)')
        if src == EnergyState.SOURCE_SYNC:
            self.press_t.clear()
            if self.bits is not None:
                self.get_logger().warn('board resync (reconnect or lost messages): re-reading '
                                       'its capabilities')
                self.connected = False      # _link re-reads capabilities, and the arm state
            self.enabled = False            # a (re)connected board starts disabled
            self._drop_software('board resync')
        if src == EnergyState.SOURCE_TAKEOVER or (
                src == EnergyState.SOURCE_FOOTSWITCH and self.bits is not None and bits & ~self.bits):
            if self.enabled:
                self.get_logger().warn('pedal pressed: the board ended software control; '
                                       'enable again deliberately to resume')
            self.enabled = False
            self._drop_software('pedal takeover', send=False)
        self.bits = bits
        self._publish_state(bits, src, suspect)

    def _publish_state(self, bits, src, suspect):
        dev = EnergyState(monopolar_cut=bool(bits & 1), monopolar_coag=bool(bits & 2),
                          bipolar=bool(bits & 4), source=src, software_enabled=self.enabled,
                          board_connected=self.connected, timing_suspect=suspect)
        dev.header.stamp = self.get_clock().now().to_msg()
        self.pub_dev.publish(dev)
        st = EnergyState(**{f: getattr(dev, f) for f in (
            'header', 'monopolar_cut', 'monopolar_coag', 'bipolar', 'source', 'software_enabled',
            'board_connected', 'timing_suspect')})
        for out, active in ((MONO, dev.monopolar_cut or dev.monopolar_coag), (BI, dev.bipolar)):
            if active:
                st.active_arms.append(self.mapping.get(out, ('', ''))[0])
        self.pub_state.publish(st)

    # ---- enable ----
    def _on_enable(self, m):
        if not m.data:
            self._drop_software('disabled by request')
            self._call(self.cli_arm, False, lambda ok: self._set_enabled(False, ok), 'disable')
            return
        if not self.connected:
            self.get_logger().error('enable refused: no board')
            return
        self._call(self.cli_arm, True, lambda ok: self._set_enabled(True, ok), 'enable')

    def _set_enabled(self, want, ok):
        if want:
            self.enabled = ok
            self.get_logger().info('software energy ENABLED' if ok else
                                   'enable refused by the board (a pedal held, or a command '
                                   'still active)')
        else:
            self.enabled = False
            self.get_logger().info('software energy disabled')
        self._publish_state(self.bits or 0, EnergyState.SOURCE_SYNC, False)

    # ---- software energy ----
    def _resolve(self, m):
        if m.arm:
            hit = [out for out, (arm, _) in self.mapping.items() if arm == m.arm]
            if not hit:
                return None, f'{m.arm} is fed by no ESU output (tool_info: no energy instrument, '\
                    'or the routing is ambiguous)'
            out = hit[0]
        elif m.output in (MONO, BI):
            out = m.output
        else:
            return None, f'unknown output {m.output}'
        if out == BI:
            ch = 'bipolar'
        elif m.mode == EnergyCommand.CUT:
            ch = 'cut'
        elif m.mode == EnergyCommand.COAG:
            ch = 'coag'
        else:
            return None, 'BLEND is chosen on the ESU front panel and fires on CUT; send CUT'
        if ch not in (self.caps or {}).get('channels', []):
            return None, f'the board has no {ch} channel'
        return dict(channel=ch, output=out, mode=EnergyCommand.COAG if out == BI else m.mode,
                    arm=self.mapping.get(out, ('', ''))[0], source=m.source), None

    def _on_target(self, m):
        if not m.activate:
            if self.want:
                self._drop_software('target released')
            return
        req, why = self._resolve(m)
        if req is None:
            self.get_logger().error(f'energy target refused: {why}',
                                    throttle_duration_sec=1.0)
            return
        if not self.enabled:
            self.get_logger().error('energy target refused: software energy is not enabled',
                                    throttle_duration_sec=1.0)
            return
        if self.want and self.want['channel'] != req['channel']:
            self.get_logger().error(f'energy target refused: {self.want["channel"]} is on; one '
                                    'software output at a time', throttle_duration_sec=1.0)
            return
        req['t'] = self._now()
        self.want = req

    def _tick(self):
        now = self._now()
        if self.pending:
            fut, sent, kind = self.pending
            if not fut.done() and now - sent > self.call_timeout:
                fut.cancel()
                self.pending = None
                self._renewal_result(False, kind)
            elif not fut.done():
                return
        if self.want and now - self.want['t'] > self.target_timeout:
            self._drop_software('target went stale')
        if not self.want or not self.connected:
            return
        ch = self.want['channel']
        if self.on != ch:
            self._send(ch, True, 'on')
        elif now - self.last_renew >= self.caps['refresh_ms'] / 1000.0:
            self._send(ch, True, 'renew')

    def _send(self, ch, on, kind):
        fut = self.cli[ch].call_async(SetBool.Request(data=on))
        self.pending = (fut, self._now(), kind)
        self.last_renew = self._now()
        if kind == 'on':
            self.on = ch
            self._publish_command(EnergyCommand.ACTIVATE, self.want)
        fut.add_done_callback(lambda f: self._done(f, kind))

    def _done(self, fut, kind):
        if self.pending and self.pending[0] is fut:
            self.pending = None
        if fut.cancelled():
            return
        r = fut.result()
        self._renewal_result(bool(r and r.success), kind)

    def _renewal_result(self, ok, kind):
        if ok:
            self.missed = 0
            return
        if kind == 'on':
            self.get_logger().error('the board refused ON (not enabled, or a pedal pressed)')
            self.enabled = False
            self._drop_software('ON refused', send=False)
            return
        self.missed += 1
        if self.missed >= 2:
            self.get_logger().error('two renewals in a row failed: software energy disabled; '
                                    'enable again deliberately')
            self.enabled = False
            self._drop_software('renewals failed')

    def _drop_software(self, why, send=True):
        if self.on and send and self.connected:
            self.cli[self.on].call_async(SetBool.Request(data=False))
        if self.on:
            self._publish_command(EnergyCommand.DEACTIVATE, self.want)
            self.get_logger().info(f'software {self.on} off: {why}')
        self.on, self.want, self.pending, self.missed = None, None, None, 0

    def _publish_command(self, action, w):
        lease = (self.caps or {}).get('lease_ms', 0)
        m = EnergyCommand(action=action, output=w['output'], mode=w['mode'], source=w['source'],
                          arm=w['arm'], expiry=Duration(sec=lease // 1000,
                                                        nanosec=lease % 1000 * 1000000))
        m.header.stamp = self.get_clock().now().to_msg()
        self.pub_cmd.publish(m)

    def _call(self, client, data, then, what):
        fut = client.call_async(SetBool.Request(data=data))

        def done(f):
            ok = (not f.cancelled()) and f.result() is not None and f.result().success
            if not ok:
                self.get_logger().warn(f'{what}: the board said no or did not answer')
            then(ok)
        fut.add_done_callback(done)

    def shutdown(self):
        """Best effort: switch software energy off and disable before exiting."""
        futs = []
        if self.on and self.connected:
            futs.append(self.cli[self.on].call_async(SetBool.Request(data=False)))
        if self.connected:
            futs.append(self.cli_arm.call_async(SetBool.Request(data=False)))
        for f in futs:
            rclpy.spin_until_future_complete(self, f, timeout_sec=0.5)


def main():
    rclpy.init()
    node = Esu()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
