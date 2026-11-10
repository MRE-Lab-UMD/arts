#!/usr/bin/env python3
"""Fake ESU-TAP board for mock tests: the board's ROS surface and its control rules, no hardware.

Stands in for the real board + micro_ros_agent so the esu node can be tested on mock. It offers
what the unmodified firmware (protocol 2) offers under /esu and follows the firmware's control
rules (shared/control.hpp), re-derived: software commands need the board armed ("enable" in
ARTS), expire lease_ms after their last renewal, and are cleared with the arm by any pedal press
(a takeover, source 3, when it releases a software channel); outputs open before others close
(break-before-make, one 10 ms cycle); state is published on change, with a snapshot (source 2)
at start. Not modelled: debounce, reconnects, the watchdog, relays.

Test hooks (parameters): pedals (string array of held pedal channels, e.g. ['cut']),
hold_delivery (true: queue state messages; false: send the queue at once, which reproduces a
press and release arriving together), board (v1 | v2 | v3).

Run it on rmw_fastrtps_cpp, as the esu node runs (services do not cross DDS vendors).
"""
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import SetBool, Trigger

CHANNELS = ('cut', 'coag', 'bipolar', 'ligasure')
COUNT = {'v1': 2, 'v2': 3, 'v3': 4}
LEASE_S = 0.5


class FakeEsuTap(Node):

    def __init__(self):
        super().__init__('fake_esu_tap')
        self.board = self.declare_parameter('board', 'v2').value
        self.declare_parameter('pedals', [''])
        self.declare_parameter('hold_delivery', False)
        self.ch = CHANNELS[:COUNT[self.board]]
        self.armed = False
        self.cmd = dict.fromkeys(self.ch, False)
        self.renewed = dict.fromkeys(self.ch, 0.0)
        self.sensed = dict.fromkeys(self.ch, False)
        self.eff = dict.fromkeys(self.ch, False)
        self.desired = dict.fromkeys(self.ch, False)
        self.cause = dict.fromkeys(self.ch, 1)
        self.queue = []
        self.pub = self.create_publisher(Int32, '/esu/activation_state', 100)
        self.srv = [self.create_service(SetBool, f'/esu/{c}', lambda q, r, c=c: self._set(c, q, r))
                    for c in self.ch]
        self.srv.append(self.create_service(SetBool, '/esu/arm', self._arm))
        self.srv.append(self.create_service(Trigger, '/esu/capabilities', self._caps))
        self._publish(2)
        self.create_timer(0.01, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _disarm(self):
        self.armed = False
        self.cmd = dict.fromkeys(self.ch, False)

    def _expire(self):
        if any(self.cmd[c] and self._now() - self.renewed[c] >= LEASE_S for c in self.ch):
            self._disarm()

    def _arm(self, q, r):
        self._expire()
        if not q.data:
            self._disarm()
            r.success = True
        else:
            r.success = not any(self.sensed.values()) and not any(self.cmd.values())
            self.armed = self.armed or r.success
        return r

    def _set(self, c, q, r):
        self._expire()
        if not q.data:
            self.cmd[c] = False
            r.success = True
        else:
            r.success = self.armed
            if r.success:
                self.cmd[c], self.renewed[c] = True, self._now()
        return r

    def _caps(self, q, r):
        r.success = True
        r.message = json.dumps(dict(
            protocol=2, firmware='3.2.0-bench-fake', board=self.board, channel_count=len(self.ch),
            watchdog=self.board != 'v1', baud=115200, lease_ms=int(LEASE_S * 1000),
            refresh_ms=100, offline_pedals=True, requires_arm=True, pedal_connected=True,
            armed=self.armed, state=self._bits(), boot_id=1, dropped_events=0))
        return r

    def _bits(self):
        return sum(int(self.eff[c]) << i for i, c in enumerate(self.ch))

    def _publish(self, source):
        self.queue.append(self._bits() | source << 4)
        if not self.get_parameter('hold_delivery').value:
            for d in self.queue:
                self.pub.publish(Int32(data=d))
            self.queue = []

    def _tick(self):
        self._expire()
        held = set(self.get_parameter('pedals').value) - {''}
        changed = {c for c in self.ch if (c in held) != self.sensed[c]}
        for c in self.ch:
            self.sensed[c] = c in held
        took_over = self.armed and any(self.sensed.values())
        if took_over:
            self._disarm()
        for c in self.ch:  # the cause is fixed when the change is asked for, as in the firmware
            want = self.sensed[c] or self.cmd[c]
            if want != self.desired.get(c, False):
                self.desired[c] = want
                self.cause[c] = 0 if c in changed else 3 if took_over else 1
        opened = takeover_sent = False
        for c in self.ch:  # break-before-make: open first, close on the next cycle
            if self.eff[c] and not self.desired[c]:
                self.eff[c] = False
                opened = True
                takeover_sent = takeover_sent or self.cause[c] == 3
                self._publish(self.cause[c])
        if not opened:
            for c in self.ch:
                if not self.eff[c] and self.desired[c]:
                    self.eff[c] = True
                    self._publish(self.cause[c])
        if took_over and not takeover_sent:
            self._publish(3)  # takeover that opened nothing: reported with the unchanged state
        if not self.get_parameter('hold_delivery').value and self.queue:
            for d in self.queue:
                self.pub.publish(Int32(data=d))
            self.queue = []


def main():
    rclpy.init()
    node = FakeEsuTap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
