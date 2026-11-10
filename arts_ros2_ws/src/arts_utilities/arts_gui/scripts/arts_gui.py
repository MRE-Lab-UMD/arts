#!/usr/bin/env python3
"""ARTS operator GUI: start only the modules an experiment needs, open the displays and RViz.

    source ~/arts/arts_ros2_ws/install/setup.bash     # the RUN rule; children inherit it
    ros2 run arts_gui arts_gui.py

A supervisor, not a node: it is never in a control or data path. Each module runs as its own
`ros2 launch arts_driver arts_bringup.launch.py <switch>:=true cart_tf:=false ...` process in
its own process group, with the same switch names as the bringup (decision 18 names), so
anything clicked here can be reproduced from a terminal with the command shown in its log.
Closing the GUI leaves the modules running; reopening it re-attaches to them (state in
~/.ros/arts_gui/state.json). `cart_tf` (the one publisher of the cart-level frames) starts
with the first module and stops with the last.

Tab "Modules": one row per module with Start/Stop, its state and its log; presets start a
whole set. The endoscope cameras are ONE process (both cameras share the Spinnaker SDK and
the recorder and display are composed into it, decisions 5 and 14), so its options -- which
cameras, hardware trigger from the sync box, the surgeon displays, recording by pedal -- are
chosen before it starts; changing them means restarting it.

Tab "Displays": where the GOOVIS (found by its EDID name) and the tower views go, the image
viewer for a single camera, and RViz with a config generated from the running arms and
cameras.

Tab "Surgery mode": per PSM, MIS (the RCM on, at the cannula) or open surgery (the RCM off),
switched HERE, at the supervisor station, never from the master (decision 17, 2026-11-03
amendment). "MIS: RCM on here" fixes the point on the arm's current shaft (the controller
refuses unless the shaft already passes within 2 mm of it, so the button never pulls the
arm); "Open surgery: RCM off" is refused while that arm's teleop is engaged and asks the
supervisor to confirm the instrument is clear of the cannula. The teleop follower disengages
on either switch, so the operator re-aligns through the gate. For this the GUI keeps one small
ROS node in a background thread; it only reads state and calls set_rcm on a click, so it is
still never in a control or data path.

The same tab has an Energy box: the ESU-TAP board's state (/esu/state: connected, software
energy enabled, which output is active and the arm it feeds) and Enable / Disable for software
energy (/esu/enable, one message per click, after a confirmation). Pedal energy never needs it,
and any ESU pedal press disables software energy on the board. ARM HOLD does not stop the ESU:
its own power switch does.

Stopping a REAL arm asks first: the pendant program must be stopped before the driver.
"""
import datetime
import json
import os
import re
import signal
import subprocess
import sys
import time

import yaml
from ament_index_python.packages import get_package_share_directory
import threading

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QTabWidget, QVBoxLayout,
    QWidget)

class RosLink:
    """The GUI's one ROS node, spun in a background thread: per PSM, rcm_state and
    teleop_state in, set_rcm out; /esu/state in, /esu/enable out (Surgery mode tab). The esu
    node runs on Fast DDS, so the ESU side is topics only (decision 18, 2026-11-03)."""

    ARMS = ('psm_left', 'psm_right')

    def __init__(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool
        from arts_interfaces.msg import EnergyState, RcmState, TeleopState
        from arts_interfaces.srv import SetRcm
        self.Bool = Bool
        self.SetRcm = SetRcm
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('arts_gui', context=self.context)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.rcm, self.teleop, self.reply, self.clients = {}, {}, {}, {}
        for arm in self.ARMS:
            self.node.create_subscription(
                RcmState, f'/{arm}/rcm_state', lambda m, a=arm: self._stamp(self.rcm, a, m), 10)
            self.node.create_subscription(
                TeleopState, f'/{arm}/teleop_state',
                lambda m, a=arm: self.teleop.__setitem__(a, m), latched)
            self.clients[arm] = self.node.create_client(SetRcm, f'/{arm}/set_rcm')
        self.esu = None
        self.node.create_subscription(EnergyState, '/esu/state',
                                      lambda m: setattr(self, 'esu', (time.monotonic(), m)), 10)
        self.esu_enable = self.node.create_publisher(Bool, '/esu/enable', 10)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:  # noqa: BLE001 -- shutdown while spinning
            pass

    @staticmethod
    def _stamp(store, arm, msg):
        store[arm] = (time.monotonic(), msg)

    def rcm_state(self, arm):
        """(age s, RcmState) or None."""
        v = self.rcm.get(arm)
        return None if v is None else (time.monotonic() - v[0], v[1])

    def engaged(self, arm):
        m = self.teleop.get(arm)
        return bool(m and m.clutch_engaged)

    def set_rcm(self, arm, enable):
        """Asynchronous; the reply text lands in self.reply[arm]."""
        c = self.clients[arm]
        if not c.service_is_ready():
            self.reply[arm] = f'/{arm}/set_rcm is not available (is the arm running?)'
            return
        req = self.SetRcm.Request(enable=enable, from_current_shaft=enable, insertion_depth=0.0)
        self.reply[arm] = 'waiting for the controller ...'
        c.call_async(req).add_done_callback(
            lambda f, a=arm: self.reply.__setitem__(
                a, ('OK: ' if f.result().success else 'REFUSED: ') + f.result().message))

    def esu_state(self):
        """(age s, EnergyState) or None: on every change and a 1 Hz snapshot."""
        return None if self.esu is None else (time.monotonic() - self.esu[0], self.esu[1])

    def set_energy(self, enable):
        self.esu_enable.publish(self.Bool(data=enable))

    def shutdown(self):
        try:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
            self.context.try_shutdown()
        except Exception:  # noqa: BLE001
            pass


STATE_DIR = os.path.expanduser('~/.ros/arts_gui')
STATE_FILE = os.path.join(STATE_DIR, 'state.json')
SETTINGS_FILE = os.path.join(STATE_DIR, 'settings.json')
STOP_ESCALATION_S = (10.0, 5.0)  # SIGINT -> SIGTERM after 10 s -> SIGKILL 5 s later

# key: (group, label, bringup switches it turns on, is a real arm)
GUARDED_ARMS = ('psm_left', 'psm_right', 'ecm')
MODULES = {
    'psm_left': ('Patient cart', 'PSM left  (UR5e + adapter)', ['psm_left'], True),
    'psm_right': ('Patient cart', 'PSM right (UR5e + adapter)', ['psm_right'], True),
    'ecm': ('Patient cart', 'ECM arm   (UR5)', ['ecm'], True),
    'psm_left_tool_actuator': ('Patient cart', 'PSM left adapter discs (Dynamixel)',
                               ['psm_left_tool_actuator'], False),
    'psm_right_tool_actuator': ('Patient cart', 'PSM right adapter discs (Dynamixel)',
                                ['psm_right_tool_actuator'], False),
    'ecm_stereo': ('Endoscope', 'Endoscope cameras', [], False),   # switches from options
    'mtm': ('Surgeon console', 'MTMs (Haply)', [], False),          # switches from options
    'footswitch_teleop': ('Surgeon console', 'Teleop footswitch (Savant)',
                          ['footswitch_teleop'], False),
    'head_sensor': ('Surgeon console', 'GOOVIS head sensor', ['head_sensor'], False),
    'gamepad': ('Surgeon console', 'Gamepad (Xbox) jog', ['gamepad'], False),
    'esu': ('Support tower', 'ESU (ESU-TAP board + esu node)', ['esu'], False),
}
PRESETS = [
    ('PSM left', ['psm_left']),
    ('PSM right', ['psm_right']),
    ('Left camera', [('ecm_stereo', dict(left=True, right=False, sync=False))]),
    ('Stereo', [('ecm_stereo', dict(left=True, right=True, sync=False))]),
    ('Stereo + sync', [('ecm_stereo', dict(left=True, right=True, sync=True))]),
    ('ECM + stereo + sync', ['ecm', ('ecm_stereo', dict(left=True, right=True, sync=True))]),
    ('Teleop left', ['psm_left', ('mtm', dict(left=True, right=False)), 'footswitch_teleop',
                     ('ecm_stereo', dict(left=True, right=True, sync=True, goovis=True))]),
    ('Teleop both', ['psm_left', 'psm_right', ('mtm', dict(left=True, right=True)),
                     'footswitch_teleop',
                     ('ecm_stereo', dict(left=True, right=True, sync=True, goovis=True))]),
]


def monitors():
    """[(output, x, y, w, h, edid_name)] from xrandr; [] without it."""
    try:
        listing = subprocess.run(['xrandr', '--listmonitors'], capture_output=True, text=True,
                                 timeout=3).stdout
        verbose = subprocess.run(['xrandr', '--verbose'], capture_output=True, text=True,
                                 timeout=3).stdout
    except (OSError, subprocess.TimeoutExpired):
        return []
    names, out, edid, in_edid = {}, None, '', False
    for line in verbose.splitlines():
        m = re.match(r'^(\S+) connected', line)
        if m:
            out = m.group(1)
            continue
        if 'EDID:' in line:
            in_edid, edid = True, ''
            continue
        if in_edid:
            if re.match(r'^\s+[0-9a-f]{32}\s*$', line):
                edid += line.strip()
                continue
            in_edid = False
            b = bytes.fromhex(edid)
            for i in range(54, 126, 18):  # EDID display descriptors; 0xFC = monitor name
                if len(b) >= i + 18 and b[i:i + 3] == b'\x00\x00\x00' and b[i + 3] == 0xfc:
                    names[out] = b[i + 5:i + 18].decode(errors='ignore').strip()
    result = []
    for line in listing.splitlines():
        m = re.match(r'^\s*\d+: \+?\*?(\S+) (\d+)/\d+x(\d+)/\d+\+(\d+)\+(\d+)', line)
        if m:
            o = m.group(1)
            result.append((o, int(m.group(4)), int(m.group(5)), int(m.group(2)),
                           int(m.group(3)), names.get(o, '')))
    return result


class Module:
    """One supervised `ros2 launch` process group."""

    def __init__(self, key):
        self.key = key
        self.pid = None
        self.args = []
        self.log = os.path.join(STATE_DIR, f'{key}.log')
        self.stop_started = None
        self.exit_note = ''

    def alive(self):
        if self.pid is None:
            return False
        try:
            os.kill(self.pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        try:  # a reused pid is not ours
            with open(f'/proc/{self.pid}/cmdline', 'rb') as f:
                return b'ros2' in f.read()
        except OSError:
            return False

    def start(self, args):
        os.makedirs(STATE_DIR, exist_ok=True)
        with open(self.log, 'a') as log:
            log.write(f'\n===== {datetime.datetime.now().isoformat(timespec="seconds")} '
                      f'start: {" ".join(args)}\n')
            log.flush()
            p = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL, start_new_session=True)
        self.pid, self.args, self.stop_started, self.exit_note = p.pid, args, None, ''
        self._popen = p

    def request_stop(self):
        if self.alive() and self.stop_started is None:
            self.stop_started = time.monotonic()
            self._signal(signal.SIGINT)

    def poll(self):
        """Advance a stop in progress; reap; return the state text."""
        p = getattr(self, '_popen', None)
        if p is not None and p.poll() is not None:
            # A stop we asked for is "stopped" whatever the code: a launch whose child ignores
            # SIGINT (the micro-ROS agent) is escalated to SIGTERM and exits -15.
            self.exit_note = ('stopped' if self.stop_started is not None
                              else f'exited ({p.returncode})')
            self._popen = None
        if not self.alive():
            if self.pid is not None and not self.exit_note:
                self.exit_note = 'exited'
            self.pid, self.stop_started = None, None
            return self.exit_note or 'stopped'
        if self.stop_started is not None:
            dt = time.monotonic() - self.stop_started
            if dt > sum(STOP_ESCALATION_S):
                self._signal(signal.SIGKILL)
            elif dt > STOP_ESCALATION_S[0]:
                self._signal(signal.SIGTERM)
            return 'stopping…'
        return f'running (pid {self.pid})'

    def _signal(self, sig):
        try:
            os.killpg(self.pid, sig)
        except (ProcessLookupError, PermissionError):
            pass


class LogView(QDialog):

    def __init__(self, path, parent=None):
        super().__init__(parent)
        self.setWindowTitle(os.path.basename(path))
        self.resize(1000, 600)
        self.path = path
        self.text = QPlainTextEdit(readOnly=True)
        lay = QVBoxLayout(self)
        lay.addWidget(QLabel(path))
        lay.addWidget(self.text)
        self.timer = QTimer(self, timeout=self.refresh)
        self.timer.start(1000)
        self.refresh()

    def refresh(self):
        try:
            with open(self.path, 'rb') as f:
                f.seek(0, 2)
                f.seek(max(0, f.tell() - 200_000))
                data = f.read().decode(errors='replace')
        except OSError as e:
            data = str(e)
        data = re.sub(r'\x1b\[[0-9;]*m', '', data)
        if data != self.text.toPlainText():
            self.text.setPlainText(data)
            self.text.verticalScrollBar().setValue(self.text.verticalScrollBar().maximum())


class Gui(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle('ARTS')
        self.modules = {k: Module(k) for k in
                        list(MODULES) + ['cart_tf', 'collision_guard', 'rviz', 'image_view']}
        self.rows = {}
        tabs = QTabWidget()
        tabs.addTab(self._modules_tab(), 'Modules')
        tabs.addTab(self._displays_tab(), 'Displays')
        try:
            self.ros = RosLink()
        except Exception as e:  # noqa: BLE001 -- the GUI must still manage processes
            self.ros = None
            print(f'arts_gui: no ROS link ({e}); Surgery mode tab disabled', file=sys.stderr)
        tabs.addTab(self._surgery_tab(), 'Surgery mode')
        self.setCentralWidget(tabs)
        self._load_state()
        self.timer = QTimer(self, timeout=self._tick)
        self.timer.start(300)
        self._tick()

    # ---------------------------------------------------------------- Surgery mode tab
    def _surgery_tab(self):
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addWidget(QLabel(
            'MIS = the RCM on at the cannula; open surgery = the RCM off. Switched here, per '
            'arm, at the supervisor station. The operator re-aligns through the gate after '
            'either switch.'))
        self.surgery_rows = {}
        for arm in RosLink.ARMS:
            box = QGroupBox(arm)
            g = QGridLayout(box)
            state = QLabel('-')
            reply = QLabel('')
            reply.setWordWrap(True)
            on = QPushButton('MIS: RCM on here')
            off = QPushButton('Open surgery: RCM off')
            on.clicked.connect(lambda _=False, a=arm: self._rcm_on(a))
            off.clicked.connect(lambda _=False, a=arm: self._rcm_off(a))
            g.addWidget(state, 0, 0, 1, 2)
            g.addWidget(on, 1, 0)
            g.addWidget(off, 1, 1)
            g.addWidget(reply, 2, 0, 1, 2)
            if self.ros is None:
                on.setEnabled(False)
                off.setEnabled(False)
            lay.addWidget(box)
            self.surgery_rows[arm] = (state, reply)
        box = QGroupBox('Energy (ESU)')
        g = QGridLayout(box)
        self.esu_label = QLabel('-')
        self.esu_label.setWordWrap(True)
        en = QPushButton('Enable software energy')
        dis = QPushButton('Disable software energy')
        en.clicked.connect(self._energy_on)
        dis.clicked.connect(lambda: self.ros.set_energy(False))
        g.addWidget(self.esu_label, 0, 0, 1, 2)
        g.addWidget(en, 1, 0)
        g.addWidget(dis, 1, 1)
        g.addWidget(QLabel('Pedal energy needs no enable. Any ESU pedal press disables software '
                           'energy. ARM HOLD does not stop the ESU: its power switch does.'),
                    2, 0, 1, 2)
        if self.ros is None:
            en.setEnabled(False)
            dis.setEnabled(False)
        lay.addWidget(box)
        lay.addStretch(1)
        return w

    def _energy_on(self):
        if QMessageBox.question(
                self, 'Energy', 'Allow SOFTWARE to activate the ESU (a teleop or policy energy '
                'request)? Pedals work regardless; any pedal press disables this again.'
                ) != QMessageBox.Yes:
            return
        self.ros.set_energy(True)

    def _rcm_on(self, arm):
        if QMessageBox.question(
                self, 'MIS', f'{arm}: fix the RCM on the current shaft (the instrument must be '
                'inserted through its cannula)?') != QMessageBox.Yes:
            return
        self.ros.set_rcm(arm, True)

    def _rcm_off(self, arm):
        if self.ros.engaged(arm):
            QMessageBox.warning(self, 'Open surgery', f'{arm}: teleop is engaged. The operator '
                                'disengages first (clutch pedal).')
            return
        if QMessageBox.question(
                self, 'Open surgery', f'{arm}: turn the RCM off? Confirm the instrument is clear '
                'of the cannula: without the RCM the shaft can move sideways.') != QMessageBox.Yes:
            return
        self.ros.set_rcm(arm, False)

    def _surgery_tick(self):
        if self.ros is None:
            return
        for arm, (state, reply) in self.surgery_rows.items():
            r = self.ros.rcm_state(arm)
            if r is None or r[0] > 1.0:
                text = 'no rcm_state (arm not running)'
            elif r[1].enabled:
                text = (f'MIS: RCM ON, deviation {r[1].deviation * 1e3:.3f} mm, insertion '
                        f'{r[1].insertion_depth * 1e3:.1f} mm')
            else:
                text = 'OPEN SURGERY: RCM off'
            text += ' | teleop ' + ('ENGAGED' if self.ros.engaged(arm) else 'disengaged')
            state.setText(text)
            reply.setText(self.ros.reply.get(arm, ''))
        e = self.ros.esu_state()
        if e is None or e[0] > 3.0:
            self.esu_label.setText('no /esu/state (ESU module not running?)')
        else:
            m = e[1]
            active = [n for n, on in (('monopolar CUT', m.monopolar_cut),
                                      ('monopolar COAG', m.monopolar_coag),
                                      ('bipolar', m.bipolar)) if on]
            self.esu_label.setText(
                f'board {"connected" if m.board_connected else "NOT CONNECTED"} | software '
                f'energy {"ENABLED" if m.software_enabled else "disabled"} | active: '
                + (', '.join(active) + f' -> {list(m.active_arms)}' if active else 'none'))

    # ---------------------------------------------------------------- Modules tab
    def _modules_tab(self):
        w = QWidget()
        lay = QVBoxLayout(w)
        top = QHBoxLayout()
        self.mock = QCheckBox('Mock hardware (arms only)')
        top.addWidget(self.mock)
        top.addStretch()
        stop_all = QPushButton('Stop all')
        stop_all.clicked.connect(self.stop_all)
        top.addWidget(stop_all)
        lay.addLayout(top)

        presets = QGroupBox('Presets (start the set; already-running modules stay)')
        pl = QGridLayout(presets)
        for i, (name, items) in enumerate(PRESETS):
            b = QPushButton(name)
            b.clicked.connect(lambda _=False, items=items: self.apply_preset(items))
            pl.addWidget(b, i // 4, i % 4)
        lay.addWidget(presets)

        groups = {}
        for key, (group, label, _, _) in MODULES.items():
            if group not in groups:
                box = QGroupBox(group)
                groups[group] = QGridLayout(box)
                lay.addWidget(box)
            g = groups[group]
            r = g.rowCount()
            g.addWidget(QLabel(label), r, 0)
            opts = QWidget()
            ol = QHBoxLayout(opts)
            ol.setContentsMargins(0, 0, 0, 0)
            g.addWidget(opts, r, 1)
            btn = QPushButton('Start')
            btn.clicked.connect(lambda _=False, k=key: self.toggle(k))
            state = QLabel('stopped')
            state.setMinimumWidth(170)
            log = QPushButton('Log')
            log.clicked.connect(lambda _=False, k=key: LogView(self.modules[k].log, self).show())
            g.addWidget(btn, r, 2)
            g.addWidget(state, r, 3)
            g.addWidget(log, r, 4)
            self.rows[key] = (btn, state, ol)

        # Options of the two composite modules.
        ol = self.rows['ecm_stereo'][2]
        self.cam_left = QCheckBox('left', checked=True)
        self.cam_right = QCheckBox('right', checked=True)
        self.cam_sync = QCheckBox('hardware trigger (sync box)', checked=True)
        self.cam_record = QCheckBox('record by pedal')
        for c in (self.cam_left, self.cam_right, self.cam_sync, self.cam_record):
            ol.addWidget(c)
        self.bag_dir = QLineEdit(os.path.expanduser('~/arts_bags'))
        self.bag_dir.setToolTip('Image bags go to <dir>/<timestamp>; the recorder starts paused '
                                'and the Savant record pedal toggles it.')
        ol.addWidget(self.bag_dir)
        # Per PSM: the instrument (arts_driver config/instruments.yaml). It sets the arm's tip
        # preset and starts it with the wrist (wrist:=true) whenever the instrument has one, and
        # gives the disc module the instrument's kinematics file, so both always agree.
        with open(os.path.join(get_package_share_directory('arts_driver'), 'config',
                               'instruments.yaml')) as fh:
            self.instruments = [k for k in yaml.safe_load(fh) or {} if k != 'stereo_laparoscope']
        self.tool = {}
        for psm in ('psm_left', 'psm_right'):
            combo = QComboBox()
            combo.addItems(self.instruments)
            combo.setToolTip('Instrument on this adapter: the arm and its discs both use it. '
                             'Change it only with the arm and its discs stopped.')
            self.tool[psm] = combo
            self.rows[psm][2].addWidget(QLabel('instrument'))
            self.rows[psm][2].addWidget(combo)
        ol = self.rows['mtm'][2]
        self.mtm_left = QCheckBox('left', checked=True)
        self.mtm_right = QCheckBox('right', checked=True)
        ol.addWidget(self.mtm_left)
        ol.addWidget(self.mtm_right)
        lay.addStretch()
        return w

    # ---------------------------------------------------------------- Displays tab
    def _displays_tab(self):
        w = QWidget()
        lay = QVBoxLayout(w)
        box = QGroupBox('Surgeon displays (inside the camera process: applied when the cameras '
                        'start; both cameras needed)')
        g = QGridLayout(box)
        self.mons = monitors()
        labels = [f'{o}  {wd}x{h}+{x}+{y}  {name}' for o, x, y, wd, h, name in self.mons]
        self.goovis = QCheckBox('GOOVIS side-by-side')
        self.goovis_out = QComboBox()
        self.goovis_out.addItems(labels)
        self.tower_out = QComboBox()
        self.tower_out.addItems(labels)
        with open(os.path.join(get_package_share_directory('arts_gui'), 'config',
                               'displays.yaml')) as f:
            disp = yaml.safe_load(f)
        try:
            with open(SETTINGS_FILE) as f:
                saved = json.load(f)
        except (OSError, ValueError):
            saved = {}
        outputs = [m[0] for m in self.mons]
        goovis = saved.get('goovis_output') or next(
            (m[0] for m in self.mons if disp['goovis_edid_contains'].upper() in m[5].upper()), '')
        tower = saved.get('tower_output') or disp.get('tower_output', '')
        if goovis in outputs:
            self.goovis_out.setCurrentIndex(outputs.index(goovis))
        if tower in outputs:
            self.tower_out.setCurrentIndex(outputs.index(tower))
        else:
            self.tower_out.setCurrentIndex(-1)  # no guess: an unset tower must be chosen
        self.goovis_out.currentIndexChanged.connect(self._save_settings)
        self.tower_out.currentIndexChanged.connect(self._save_settings)
        self.tower = QCheckBox('tower view (left camera)', checked=True)
        self.fullscreen = QCheckBox('fullscreen', checked=True)
        g.addWidget(self.goovis, 0, 0)
        g.addWidget(self.goovis_out, 0, 1)
        g.addWidget(self.tower, 1, 0)
        g.addWidget(self.tower_out, 1, 1)
        g.addWidget(self.fullscreen, 2, 0)
        restart = QPushButton('Restart cameras with these settings')
        restart.clicked.connect(self.restart_cameras)
        g.addWidget(restart, 2, 1)
        lay.addWidget(box)

        box = QGroupBox('Viewers')
        hl = QHBoxLayout(box)
        iv = QPushButton('Image viewer (left camera, else right)')
        iv.clicked.connect(self.image_view)
        rv = QPushButton('RViz (running arms and cameras)')
        rv.clicked.connect(self.rviz)
        hl.addWidget(iv)
        hl.addWidget(rv)
        lay.addWidget(box)
        lay.addStretch()
        return w

    # ---------------------------------------------------------------- actions
    def _bringup(self, switches, **extra):
        args = ['ros2', 'launch', 'arts_driver', 'arts_bringup.launch.py', 'cart_tf:=false',
                'collision_guard:=false',
                f'use_fake_hardware:={"true" if self.mock.isChecked() else "false"}']
        args += [f'{s}:=true' for s in switches]
        args += [f'{k}:={v}' for k, v in extra.items()]
        return args

    def _save_settings(self, *_):
        def out(combo):
            i = combo.currentIndex()
            return self.mons[i][0] if 0 <= i < len(self.mons) else ''
        os.makedirs(STATE_DIR, exist_ok=True)
        with open(SETTINGS_FILE, 'w') as f:
            json.dump({'goovis_output': out(self.goovis_out),
                       'tower_output': out(self.tower_out)}, f)

    def _monitor_pos(self, combo):
        i = combo.currentIndex()
        return f'{self.mons[i][1]},{self.mons[i][2]}' if 0 <= i < len(self.mons) else '0,0'

    def _args_for(self, key):
        if key == 'ecm_stereo':
            sw = [s for s, c in (('ecm_stereo_left', self.cam_left),
                                 ('ecm_stereo_right', self.cam_right),
                                 ('ecm_stereo_sync', self.cam_sync)) if c.isChecked()]
            if not (self.cam_left.isChecked() or self.cam_right.isChecked()):
                raise ValueError('pick at least one camera')
            extra = {}
            both = self.cam_left.isChecked() and self.cam_right.isChecked()
            if self.goovis.isChecked() or self.tower.isChecked():
                for want, combo, what in ((self.goovis.isChecked(), self.goovis_out, 'GOOVIS'),
                                          (self.tower.isChecked(), self.tower_out, 'tower')):
                    if want and not 0 <= combo.currentIndex() < len(self.mons):
                        raise ValueError(f'choose the {what} monitor (Displays tab)')
                if both:
                    extra.update(display='true', display_hmd=str(self.goovis.isChecked()).lower(),
                                 display_fullscreen=str(self.fullscreen.isChecked()).lower(),
                                 display_hmd_pos=self._monitor_pos(self.goovis_out),
                                 display_tower_pos=self._monitor_pos(self.tower_out))
            if self.cam_record.isChecked():
                if self.modules['footswitch_teleop'].alive():
                    raise ValueError('record by pedal runs the Savant inside the camera process; '
                                     'stop the separate teleop footswitch first')
                stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                extra.update(image_bag=os.path.join(os.path.expanduser(self.bag_dir.text()),
                                                    f'images_{stamp}'),
                             image_bag_paused='true')
                sw.append('footswitch_teleop')
            return self._bringup(sw, **extra)
        if key == 'mtm':
            sw = [s for s, c in (('mtm_left', self.mtm_left), ('mtm_right', self.mtm_right))
                  if c.isChecked()]
            if not sw:
                raise ValueError('pick at least one MTM')
            return self._bringup(sw)
        if key == 'footswitch_teleop' and self.modules['ecm_stereo'].alive() \
                and 'footswitch_teleop:=true' in self.modules['ecm_stereo'].args:
            raise ValueError('the cameras are recording by pedal: the Savant already runs '
                             'inside the camera process')
        if key in ('psm_left', 'psm_right', 'psm_left_tool_actuator', 'psm_right_tool_actuator'):
            psm = key[:len('psm_right')] if key.startswith('psm_right') else 'psm_left'
            tool = self.tool[psm].currentText()
            extra = {f'{psm}_tool': tool}
            if key == psm and tool != 'none' and not self.mock.isChecked():
                extra['wrist'] = 'true'
            if key != psm:
                # tool_homing leaves the servos in its own mode; the drive writes velocity
                # mode back (an EEPROM write, refused unless allowed), as every session does.
                extra['allow_mode_write'] = 'true'
            return self._bringup(MODULES[key][2], **extra)
        return self._bringup(MODULES[key][2])

    def start(self, key):
        m = self.modules[key]
        if m.alive():
            return
        try:
            args = self._args_for(key)
        except ValueError as e:
            QMessageBox.warning(self, 'ARTS', f'{MODULES[key][1]}: {e}')
            return
        if not self.modules['cart_tf'].alive():
            self.modules['cart_tf'].start(['ros2', 'run', 'arts_driver', 'cart_tf'])
        # ONE collision guard for all arms (it follows the arms that publish a description),
        # up with the first arm, down after the last (arts_safety).
        if key in GUARDED_ARMS and not self.modules['collision_guard'].alive():
            self.modules['collision_guard'].start(
                ['ros2', 'run', 'arts_safety', 'collision_guard.py', '--ros-args',
                 '--params-file', os.path.join(get_package_share_directory('arts_safety'),
                                               'config', 'collision_guard.yaml')])
        m.start(args)
        self._save_state()

    def stop(self, key, ask=True):
        m = self.modules[key]
        if not m.alive():
            return True
        real_arm = key in MODULES and MODULES[key][3] and 'use_fake_hardware:=false' in m.args
        if real_arm and ask:
            r = QMessageBox.question(
                self, 'Stop arm', f'{MODULES[key][1]}: stop the PENDANT program first '
                '(pendant: Stop), then press Yes to stop the driver.')
            if r != QMessageBox.Yes:
                return False
        m.request_stop()
        return True

    def toggle(self, key):
        if self.modules[key].alive():
            self.stop(key)
        else:
            self.start(key)

    def apply_preset(self, items):
        for item in items:
            key, opts = (item, {}) if isinstance(item, str) else item
            if key == 'ecm_stereo' and opts:
                self.cam_left.setChecked(opts.get('left', True))
                self.cam_right.setChecked(opts.get('right', True))
                self.cam_sync.setChecked(opts.get('sync', False))
                if 'goovis' in opts:
                    self.goovis.setChecked(opts['goovis'])
            if key == 'mtm' and opts:
                self.mtm_left.setChecked(opts.get('left', True))
                self.mtm_right.setChecked(opts.get('right', True))
            self.start(key)

    def stop_all(self):
        arms = [k for k in MODULES if MODULES[k][3] and self.modules[k].alive()
                and 'use_fake_hardware:=false' in self.modules[k].args]
        if arms:
            r = QMessageBox.question(self, 'Stop all', 'Real arms running: stop their PENDANT '
                                     'programs first, then press Yes.')
            if r != QMessageBox.Yes:
                return
        for k in self.modules:
            self.stop(k, ask=False)

    def restart_cameras(self):
        m = self.modules['ecm_stereo']
        if m.alive():
            m.request_stop()
            self._restart_pending = time.monotonic()
        else:
            self.start('ecm_stereo')

    def image_view(self):
        cam = self.modules['ecm_stereo']
        topic = '/ecm/stereo_left/image_raw'
        if cam.alive() and 'ecm_stereo_left:=true' not in cam.args:
            topic = '/ecm/stereo_right/image_raw'
        m = self.modules['image_view']
        if m.alive():
            m.request_stop()
        m.start(['ros2', 'run', 'rqt_image_view', 'rqt_image_view', topic])

    def rviz(self):
        src = os.path.join(get_package_share_directory('arts_description'), 'rviz', 'arts.rviz')
        with open(src) as f:
            cfg = yaml.safe_load(f)
        displays = cfg['Visualization Manager']['Displays']
        running_arms = {k for k in ('psm_left', 'psm_right', 'ecm') if self.modules[k].alive()}
        kept = []
        for d in displays:
            topic = (d.get('Description Topic') or {}).get('Value', '')
            if d.get('Class') == 'rviz_default_plugins/RobotModel':
                arm = topic.strip('/').split('/')[0]
                if arm not in running_arms:
                    continue
            kept.append(d)
        cam = self.modules['ecm_stereo']
        for side in ('left', 'right'):
            if cam.alive() and f'ecm_stereo_{side}:=true' in cam.args:
                kept.append({'Class': 'rviz_default_plugins/Image', 'Enabled': True,
                             'Name': f'stereo_{side}', 'Value': True,
                             'Topic': {'Value': f'/ecm/stereo_{side}/image_raw',
                                       'Reliability Policy': 'Best Effort',
                                       'Durability Policy': 'Volatile', 'Depth': 1}})
        cfg['Visualization Manager']['Displays'] = kept
        os.makedirs(STATE_DIR, exist_ok=True)
        out = os.path.join(STATE_DIR, 'arts_gui.rviz')
        with open(out, 'w') as f:
            yaml.safe_dump(cfg, f)
        m = self.modules['rviz']
        if m.alive():
            m.request_stop()
            self._rviz_pending = out
            return
        m.start(['ros2', 'run', 'rviz2', 'rviz2', '-d', out])

    # ---------------------------------------------------------------- housekeeping
    def _tick(self):
        self._surgery_tick()
        for key, (btn, state, _) in self.rows.items():
            s = self.modules[key].poll()
            state.setText(s)
            running = self.modules[key].alive()
            btn.setText('Stop' if running else 'Start')
        for key in ('cart_tf', 'collision_guard', 'rviz', 'image_view'):
            self.modules[key].poll()
        if getattr(self, '_restart_pending', None) and not self.modules['ecm_stereo'].alive():
            self._restart_pending = None
            self.start('ecm_stereo')
        if getattr(self, '_rviz_pending', None) and not self.modules['rviz'].alive():
            out, self._rviz_pending = self._rviz_pending, None
            self.modules['rviz'].start(['ros2', 'run', 'rviz2', 'rviz2', '-d', out])
        # cart_tf follows the modules: up while any runs, down after the last one.
        if not any(self.modules[k].alive() for k in MODULES) and self.modules['cart_tf'].alive():
            self.modules['cart_tf'].request_stop()
        if (not any(self.modules[k].alive() for k in GUARDED_ARMS)
                and self.modules['collision_guard'].alive()):
            self.modules['collision_guard'].request_stop()
        self._save_state()

    def _save_state(self):
        os.makedirs(STATE_DIR, exist_ok=True)
        data = {k: {'pid': m.pid, 'args': m.args} for k, m in self.modules.items() if m.alive()}
        tmp = STATE_FILE + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(data, f)
        os.replace(tmp, STATE_FILE)

    def _load_state(self):
        try:
            with open(STATE_FILE) as f:
                data = json.load(f)
        except (OSError, ValueError):
            return
        for k, v in data.items():
            if k in self.modules:
                self.modules[k].pid, self.modules[k].args = v['pid'], v['args']
                if not self.modules[k].alive():
                    self.modules[k].pid, self.modules[k].args = None, []

    def closeEvent(self, event):
        running = [MODULES.get(k, (None, k))[1] for k, m in self.modules.items()
                   if m.alive() and k not in ('cart_tf', 'collision_guard')]
        if running:
            QMessageBox.information(self, 'ARTS', 'Still running (reopen the GUI to manage '
                                    'them):\n' + '\n'.join(running))
        if self.ros is not None:
            self.ros.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)
    gui = Gui()
    gui.resize(1100, 700)
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
