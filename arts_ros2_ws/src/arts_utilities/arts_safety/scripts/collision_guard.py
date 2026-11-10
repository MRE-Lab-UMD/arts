#!/usr/bin/env python3
"""Collision guard: capsule distances between the arms and within each arm; hold all arms when
anything gets too close.

    ros2 run arts_safety collision_guard.py --ros-args --params-file config/collision_guard.yaml

Every arm that publishes <arm>/robot_description is guarded. Its links are capsules FITTED at
start-up to the vendor collision meshes in that description: one capsule per link, covering
points spread over the whole mesh SURFACE (not only its vertices), the thinnest along the best
axis, each end-sphere centre pushed inward as far as coverage allows (on a joint housing's
centre, not a radius past the link's end); the tools are fixed capsules from the CAD extents
(config `tools`): the adapters and the first 150 mm of each instrument shaft, the endoscope
camera head and the first 200 mm of the scope. Instrument wrists and tips are NOT guarded (they
may touch: handoffs; their models are estimates). Poses
come from tf (root `world_frame`) at `rate` Hz. Checked pairs:
  - between arms: every capsule of one arm against every capsule of another;
  - within an arm: every pair at least three links apart in the chain (adjacent links and the
    tools against the last two wrist links touch by construction), minus pairs already closer
    than `stop_distance` at start-up (logged; a pose the arm starts in is not a collision);
  - every link from the upper arm outward, and the tools, against the pedestal spheres
    (`pedestal_spheres`: the structure holding the arms, between the mount frames).
Surface distance = segment distance - both radii. Below `stop_distance` (or a guarded arm's tf
older than `stale_s`): /arts/collision_hold = true (latched). Every servo controller holds
position on it; the teleop followers disengage until the pedal is pressed after it clears.
The hold stays until `~/release` (std_srvs/Trigger): then a recovery phase lets the operator
separate the arms (motion away is free; the hold returns if any pair below `stop_distance`
gets `recovery_slack` closer than at the release, or a new pair crosses it) and ends when every
pair is beyond `stop_distance + clear_margin`. Below `warn_distance` the pair is reported.
Publishes: /arts/collision_hold (Bool), ~/state (String, JSON: closest pairs, hold, reason),
~/capsules (MarkerArray, green / yellow below warn / red below stop) for RViz.
"""
import itertools
import json
import os
import re
import struct
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# The UR chain order (link suffixes): the in-arm pair rule counts positions in this list.
PEDESTAL = 'pedestal'            # the pseudo-arm of the pedestal spheres
CHAIN = ['base_link_inertia', 'shoulder_link', 'upper_arm_link', 'forearm_link',
         'wrist_1_link', 'wrist_2_link', 'wrist_3_link']
TOOL_INDEX = len(CHAIN)          # every tool capsule sits after wrist_3
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def read_stl(path):
    b = open(path, 'rb').read()
    if b[:5] == b'solid' and b'facet' in b[:512]:
        return np.array([[float(x) for x in ln.split()[1:4]] for ln in
                         b.decode(errors='ignore').splitlines() if ln.strip().startswith('vertex')])
    n = struct.unpack('<I', b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'),
                                                         ('a', '<u2')]))
    return a['v'].reshape(-1, 3).astype(float)


def resolve(uri):
    m = re.match(r'package://([^/]+)/(.*)', uri)
    if m:
        return os.path.join(get_package_share_directory(m.group(1)), m.group(2))
    return uri[7:] if uri.startswith('file://') else uri


def surface_points(tri_v, spacing=0.004, voxel=None):
    """Points over every triangle of a mesh (vertices + a grid of `spacing`), so a fit and its
    check see the whole surface. Vertices alone are not enough: a low-poly tube has vertices
    only at its two ends, and capsules around those leave its middle uncovered (the first
    version of this guard did exactly that, 2026-11-07). `voxel`: reduce to one point per cell."""
    tris = tri_v.reshape(-1, 3, 3)
    out = [tri_v]
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    edge = np.max([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - a, axis=1),
                   np.linalg.norm(c - b, axis=1)], axis=0)
    n_all = np.ceil(edge / spacing).astype(int)
    for n in np.unique(n_all[n_all >= 2]):       # vectorized per grid size
        sel = n_all == n
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
        m = (i + j) <= n
        u, w = i[m] / n, j[m] / n
        pa, pb, pc = a[sel], b[sel], c[sel]
        pts = (pa[:, None, :] + u[None, :, None] * (pb - pa)[:, None, :]
               + w[None, :, None] * (pc - pa)[:, None, :])
        out.append(pts.reshape(-1, 3))
    pts = np.vstack(out)
    q = voxel or 1e-5
    keys = np.floor(pts / q).astype(np.int64)
    # Exact packing (21 bits per axis), not a hash: a collision would silently drop a point.
    k = keys - keys.min(axis=0)
    if k.max() >= (1 << 21):
        raise ValueError(f'mesh too large for {q} m cells')
    _, idx = np.unique((k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2], return_index=True)
    return pts[idx] if voxel is None else (keys[idx] + 0.5) * q


def capsule_volume(a, b, r):
    return np.pi * r * r * np.linalg.norm(b - a) + 4.0 / 3.0 * np.pi * r ** 3


def capsule_along(v, u):
    """The tightest capsule around points v with its axis along u: the axis through the centre
    of the cross-section's bounding box; radius r = the largest distance rho of any point from
    that axis (no capsule along u can be thinner); then each END-SPHERE CENTRE pushed inward as
    far as every point beyond it allows: a point at axial position t, distance rho, is inside
    an end sphere centred at axial position e when |t - e| <= sqrt(r^2 - rho^2), so the lower
    centre is min(t + sqrt(r^2 - rho^2)) and the upper max(t - sqrt(r^2 - rho^2)). On a UR link
    (a rounded profile with a joint housing at each end) this puts each end sphere on the
    housing's centre instead of a radius past the link's end."""
    c = v.mean(axis=0)
    basis = np.linalg.svd(np.eye(3) - np.outer(u, u))[0][:, :2]
    p = ((v - c) - np.outer((v - c) @ u, u)) @ basis
    c = c + basis @ ((p.min(axis=0) + p.max(axis=0)) / 2)
    t = (v - c) @ u
    rho = np.linalg.norm((v - c) - np.outer(t, u), axis=1)
    r = float(rho.max())
    slack = np.sqrt(np.maximum(r * r - rho * rho, 0.0))
    lo, hi = float(np.min(t + slack)), float(np.max(t - slack))
    if lo > hi:            # shorter than its width: a sphere
        e = 0.5 * (lo + hi)
        a = b = c + e * u
        return a, b, float(np.linalg.norm(v - a, axis=1).max())
    return c + lo * u, c + hi * u, r


def fit_capsule(v):
    """The smallest-volume capsule_along over the principal axes and the frame's own x, y, z
    (UR link meshes are aligned with their link frames)."""
    c = v.mean(axis=0)
    axes = list(np.linalg.svd(v - c, full_matrices=False)[2]) + list(np.eye(3))
    return min((capsule_along(v, u) for u in axes), key=lambda cap: capsule_volume(*cap))


def fit_capsules(points, k_max=1, penalty=0.15, sample=8000):
    """1..k_max capsules covering every point: k-means chunks (seeded along the principal axis)
    of a sample, fit_capsule per chunk, each radius grown until the union covers ALL points; k
    by the smallest total volume, each extra capsule costing `penalty`."""
    rng = np.random.default_rng(0)
    sub = points[rng.choice(len(points), min(len(points), sample), replace=False)]
    c0 = sub.mean(axis=0)
    u = np.linalg.svd(sub - c0, full_matrices=False)[2][0]
    t = (sub - c0) @ u
    best = None
    for k in range(1, k_max + 1):
        centres = np.array([sub[np.argmin(np.abs(t - q))] for q in
                            np.quantile(t, (np.arange(k) + 0.5) / k)])
        for _ in range(40):
            lab = np.argmin(((sub[:, None, :] - centres[None]) ** 2).sum(-1), axis=1)
            new = np.array([sub[lab == i].mean(axis=0) if np.any(lab == i) else centres[i]
                            for i in range(k)])
            if np.allclose(new, centres, atol=1e-6):
                break
            centres = new
        caps = [fit_capsule(sub[lab == i]) for i in range(k) if np.sum(lab == i) >= 4]
        d = np.stack([seg_point_dist(a, b, points) - r for a, b, r in caps], axis=1)
        near = np.argmin(d, axis=1)
        grown = []
        for i, (a, b, r) in enumerate(caps):
            m = (near == i) & (d[:, i] > 0)
            grown.append((a, b, r + (float(d[m, i].max()) if m.any() else 0.0)))
        vol = sum(capsule_volume(*cap) for cap in grown) * (1 + penalty * (len(grown) - 1))
        if best is None or vol < best[0]:
            best = (vol, grown)
    return best[1]


def seg_point_dist(a, b, p):
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(ab @ ab, 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + np.outer(t, ab)), axis=1)


def seg_seg_dist(p1, q1, p2, q2):
    """Closest distance between segments [p1,q1] and [p2,q2], vectorized over rows (N,3)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a = np.einsum('ij,ij->i', d1, d1)
    e = np.einsum('ij,ij->i', d2, d2)
    f = np.einsum('ij,ij->i', d2, r)
    c = np.einsum('ij,ij->i', d1, r)
    b = np.einsum('ij,ij->i', d1, d2)
    denom = a * e - b * b
    eps = 1e-12
    s = np.where(denom > eps, np.clip((b * f - c * e) / np.maximum(denom, eps), 0, 1), 0.0)
    t = (b * s + f) / np.maximum(e, eps)
    # t outside [0,1]: clamp and recompute s.
    t_lo, t_hi = t < 0, t > 1
    s = np.where(t_lo, np.clip(-c / np.maximum(a, eps), 0, 1), s)
    s = np.where(t_hi, np.clip((b - c) / np.maximum(a, eps), 0, 1), s)
    t = np.clip(t, 0, 1)
    c1 = p1 + d1 * s[:, None]
    c2 = p2 + d2 * t[:, None]
    return np.linalg.norm(c1 - c2, axis=1)


class Capsule:
    def __init__(self, arm, name, index, frame_a, a, frame_b, b, r):
        self.arm, self.name, self.index, self.r = arm, name, index, r
        self.frame_a, self.a, self.frame_b, self.b = frame_a, np.asarray(a), frame_b, np.asarray(b)


class Guard(Node):

    def __init__(self):
        super().__init__('collision_guard')
        p = self.declare_parameter
        self.arms = list(p('arms', ['psm_left', 'psm_right', 'ecm']).value)
        self.world = p('world_frame', 'world').value
        self.rate = p('rate', 100.0).value
        self.stop = p('stop_distance', 0.02).value
        self.warn = p('warn_distance', 0.05).value
        self.clear_margin = p('clear_margin', 0.02).value
        self.slack = p('recovery_slack', 0.002).value
        self.stale_s = p('stale_s', 0.2).value
        self.wait_s = p('description_wait_s', 5.0).value
        self.k_max = p('capsules_per_link', 1).value
        local = os.environ.get('ARTS_LOCAL', os.path.expanduser('~/arts_local'))
        self.pedestal_mesh = p('pedestal_mesh', os.path.join(
            local, 'description_catalog', 'parts', '1_pedestal', '2_mesh', 'visual',
            'pedestal.stl')).value
        self.pedestal_rpy = list(p('pedestal_mesh_rpy', [0.0, 0.0, 1.5707963]).value)
        self.pedestal_xyz = list(p('pedestal_mesh_xyz', [0.0, 0.0, 0.0]).value)
        self.mount_clear = p('pedestal_mount_clearance', 0.12).value
        raw = p('pedestal_spheres', '[]').value
        self.sphere_cfg = json.loads(raw) if isinstance(raw, str) else raw
        self.cache_dir = os.path.expanduser(
            p('cache_dir', '~/.ros/arts_collision_guard').value)
        self.pedestal = None        # (the mesh point cloud is not used: pedestal_spheres)
        self.ped_caps = np.zeros(0, dtype=int)
        self.tools_cfg = {}
        for arm in self.arms:
            raw = p(f'tools.{arm}', '[]').value
            self.tools_cfg[arm] = json.loads(raw) if isinstance(raw, str) else raw
        self.descriptions = {}
        for arm in self.arms:
            self.create_subscription(String, f'/{arm}/robot_description',
                                     lambda m, a=arm: self.descriptions.__setitem__(a, m.data),
                                     LATCHED)
        self.tf = Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = TransformListener(self.tf, self)  # kept: else garbage-collected
        self.hold_pub = self.create_publisher(
            Bool, p('hold_topic', '/arts/collision_hold').value, LATCHED)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '~/capsules', 1)
        self.create_service(Trigger, '~/release', self._release)
        self.capsules = []
        self.pairs = None
        # Three causes, one published flag: not started yet (capsules / start-up exclusion);
        # tf missing or stale (clears by itself when tf is back: the followers re-engage only
        # on a fresh pedal press anyway); a collision (latched until ~/release).
        self.started = False
        self.tf_fault = ''
        self.collision = ''
        self.recovery = None        # pair index -> distance at release
        self.last_marker = 0.0
        self.last_membership = 0.0
        self.built_arms = set()
        self.hold = None
        self.reason = ''
        self._update_hold()
        self.t0 = time.monotonic()
        self.timer = self.create_timer(1.0 / self.rate, self._tick)

    # ---- set-up -----------------------------------------------------------------------
    def _build(self):
        arms = [a for a in self.arms if a in self.descriptions]
        caps = []
        for arm in arms:
            root = ET.fromstring(self.descriptions[arm])
            for link in root.findall('link'):
                name = link.get('name')
                suffix = name[len(arm) + 1:] if name.startswith(arm + '_') else name
                if suffix not in CHAIN:
                    continue
                verts = []
                for col in link.findall('collision'):
                    mesh = col.find('geometry/mesh')
                    if mesh is None:
                        continue
                    v = read_stl(resolve(mesh.get('filename')))
                    if mesh.get('scale'):
                        v = v * np.array([float(x) for x in mesh.get('scale').split()])
                    org = col.find('origin')
                    if org is not None:
                        xyz = np.array([float(x) for x in org.get('xyz', '0 0 0').split()])
                        rpy = [float(x) for x in org.get('rpy', '0 0 0').split()]
                        v = Rotation.from_euler('xyz', rpy).apply(v) + xyz
                    verts.append(v)
                if not verts:
                    continue
                fitted = fit_capsules(surface_points(np.vstack(verts)), k_max=self.k_max)
                for q, (a, b, r) in enumerate(fitted):
                    label = suffix if len(fitted) == 1 else f'{suffix}[{q}]'
                    caps.append(Capsule(arm, label, CHAIN.index(suffix), name, a, name, b, r))
            for t in self.tools_cfg.get(arm, []):
                if 'mesh' in t:
                    # A CAD mesh in its frame (the adapter, the endoscope camera head): fitted
                    # like a link, optionally cropped (crop_axis, crop_max: keep the points below).
                    frame = t['frame'].replace('{arm}', arm)
                    path = os.path.expanduser(t['mesh'].replace(
                        '$ARTS_LOCAL', os.environ.get('ARTS_LOCAL', '~/arts_local')))
                    if not os.path.isfile(path):
                        self.get_logger().error(f'{arm}/{t["name"]}: no mesh {path}: NOT guarded')
                        continue
                    pts = surface_points(read_stl(path), spacing=0.004, voxel=0.002)
                    if 'crop_max' in t:
                        pts = pts[pts[:, 'xyz'.index(t.get('crop_axis', 'y'))] <= t['crop_max']]
                    fitted = fit_capsules(pts, k_max=int(t.get('capsules', 1)), penalty=0.0)
                    for q, (a, b, r) in enumerate(fitted):
                        label = t['name'] if len(fitted) == 1 else f'{t["name"]}[{q}]'
                        caps.append(Capsule(arm, label, TOOL_INDEX, frame, a, frame, b, r))
                    continue
                fa = t['a_frame'].replace('{arm}', arm)
                fb = t['b_frame'].replace('{arm}', arm)
                caps.append(Capsule(arm, t['name'], TOOL_INDEX, fa, t['a'], fb, t['b'], t['r']))
        caps += self._pedestal_spheres(arms)
        self.capsules = caps
        pairs = []
        for i, j in itertools.combinations(range(len(caps)), 2):
            ci, cj = caps[i], caps[j]
            if PEDESTAL in (ci.arm, cj.arm):
                # The pedestal spheres: against links from the upper arm outward and the tools
                # (the bases and shoulders sit on the mounts), never against each other.
                other = cj if ci.arm == PEDESTAL else ci
                if other.arm == PEDESTAL or other.index < 2:
                    continue
                pairs.append((i, j))
                continue
            if ci.arm == cj.arm and abs(ci.index - cj.index) <= 2:
                continue
            pairs.append((i, j))
        self.pairs = np.array(pairs, dtype=int).reshape(-1, 2)
        self.get_logger().info(
            f'guarding {arms} (no description from '
            f'{[a for a in self.arms if a not in arms]}): {len(caps)} capsules, '
            f'{len(self.pairs)} pairs; ' + ', '.join(f'{c.arm}/{c.name} r {1e3 * c.r:.0f} mm'
                                                    for c in caps))

    def _pedestal_spheres(self, arms):
        """The pedestal where an arm can fold into it: the structure holding the three arms.
        Each sphere (config `pedestal_spheres`) is centred midway between two mount frames
        (<arm>_mount, tf from cart_tf); its diameter is the distance between them times
        `diameter_scale`, or a fixed `diameter` (m); `offset` (m, world xyz) moves the centre."""
        out = []
        for sp in self.sphere_cfg:
            pts = []
            for frame in sp['between']:
                try:
                    tr = self.tf.lookup_transform(self.world, frame, Time())
                except Exception as e:  # noqa: BLE001
                    self.get_logger().error(f'pedestal sphere {sp["name"]}: no tf {frame} '
                                            f'({type(e).__name__}): NOT guarded')
                    pts = None
                    break
                t = tr.transform.translation
                pts.append(np.array([t.x, t.y, t.z]))
            if pts is None:
                continue
            c = 0.5 * (pts[0] + pts[1]) + np.array(sp.get('offset', [0.0, 0.0, 0.0]), float)
            dia = sp.get('diameter', np.linalg.norm(pts[1] - pts[0]) * sp.get('diameter_scale', 1.0))
            out.append(Capsule(PEDESTAL, sp['name'], -1, self.world, c, self.world, c, 0.5 * dia))
            self.get_logger().info(f'pedestal sphere {sp["name"]}: centre {np.round(c, 3).tolist()}'
                                   f', diameter {1e3 * dia:.0f} mm')
        return out

    def _build_pedestal(self, arms):
        """The pedestal (the imported Vention mesh, placed in world as cart_tf places it) as a
        1 cm point cloud (cached per mesh file), minus the points within
        `pedestal_mount_clearance` of each guarded arm's mount (the base sits there by design).
        No mesh: not guarded (logged)."""
        from scipy.spatial import cKDTree
        self.pedestal = None
        if not self.pedestal_mesh or not os.path.isfile(self.pedestal_mesh):
            self.get_logger().warn(f'no pedestal mesh at {self.pedestal_mesh!r}: the pedestal is '
                                   'NOT guarded')
            return
        cache = os.path.join(self.cache_dir,
                             f'pedestal_{int(os.path.getmtime(self.pedestal_mesh))}.npy')
        if os.path.isfile(cache):
            pts = np.load(cache)
        else:
            pts = surface_points(read_stl(self.pedestal_mesh), spacing=0.01, voxel=0.01)
            try:
                os.makedirs(os.path.dirname(cache), exist_ok=True)
                np.save(cache, pts)
            except OSError:
                pass
        pts = Rotation.from_euler('xyz', self.pedestal_rpy).apply(pts) + self.pedestal_xyz
        for arm in arms:
            try:   # static (cart_tf), already received: never wait inside the timer callback
                tr = self.tf.lookup_transform(self.world, f'{arm}_mount', Time())
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'no tf for {arm}_mount ({type(e).__name__}): its mount '
                                       'area is not cleared from the pedestal')
                continue
            t = tr.transform.translation
            pts = pts[np.linalg.norm(pts - np.array([t.x, t.y, t.z]), axis=1) > self.mount_clear]
        self.pedestal = cKDTree(pts)
        self.get_logger().info(f'pedestal guarded: {len(pts)} points (1 cm) from '
                               f'{self.pedestal_mesh}')

    def _pedestal_distances(self, A, B, r):
        """Surface distance of each checked capsule to the pedestal: the nearest pedestal point
        to samples every 5 mm along the capsule's axis, minus the radius, half the sample
        spacing and half a cell diagonal (so the estimate never overstates the gap)."""
        out = np.zeros(len(self.ped_caps))
        for n, k in enumerate(self.ped_caps):
            m = max(2, int(np.ceil(np.linalg.norm(B[k] - A[k]) / 0.005)) + 1)
            seg = A[k] + np.outer(np.linspace(0.0, 1.0, m), B[k] - A[k])
            out[n] = self.pedestal.query(seg)[0].min() - r[k] - 0.0025 - 0.0087
        return out

    def _exclude_start_contacts(self, d):
        npairs = len(self.pairs)
        if len(self.ped_caps):
            keep_p = d[npairs:] >= self.stop
            for k in np.nonzero(~keep_p)[0]:
                c = self.capsules[self.ped_caps[k]]
                self.get_logger().warn(f'excluded {c.arm}: {c.name} - pedestal '
                                       f'({1e3 * d[npairs + k]:.0f} mm at start-up)')
            self.ped_caps = self.ped_caps[keep_p]
        keep = np.ones(len(self.pairs), bool)
        for k, (i, j) in enumerate(self.pairs):
            ci, cj = self.capsules[i], self.capsules[j]
            if ci.arm == cj.arm and d[k] < self.stop:
                keep[k] = False
                self.get_logger().warn(
                    f'excluded {ci.arm}: {ci.name} - {cj.name} ({1e3 * d[k]:.0f} mm at start-up)')
        self.pairs = self.pairs[keep]

    # ---- per cycle --------------------------------------------------------------------
    def _points(self):
        """World endpoints of every capsule, or (None, why) if a transform is missing/stale."""
        now = self.get_clock().now()
        cache = {}
        A = np.zeros((len(self.capsules), 3))
        B = np.zeros((len(self.capsules), 3))
        for k, c in enumerate(self.capsules):
            for frame, local, out in ((c.frame_a, c.a, A), (c.frame_b, c.b, B)):
                if frame not in cache:
                    try:
                        tr = self.tf.lookup_transform(self.world, frame, Time())
                    except Exception as e:  # noqa: BLE001 (any tf failure is a hold)
                        return None, None, f'no tf {self.world} <- {frame} ({type(e).__name__})'
                    age = (now - Time.from_msg(tr.header.stamp)).nanoseconds * 1e-9
                    if tr.header.stamp.sec != 0 and age > self.stale_s:
                        return None, None, f'tf {frame} {age:.2f} s old'
                    q = tr.transform.rotation
                    t = tr.transform.translation
                    cache[frame] = (Rotation.from_quat([q.x, q.y, q.z, q.w]),
                                    np.array([t.x, t.y, t.z]))
                rot, pos = cache[frame]
                out[k] = rot.apply(local) + pos
        return A, B, ''

    def _distances(self):
        A, B, why = self._points()
        if A is None:
            return None, None, None, why
        i, j = self.pairs[:, 0], self.pairs[:, 1]
        r = np.array([c.r for c in self.capsules])
        d = seg_seg_dist(A[i], B[i], A[j], B[j]) - r[i] - r[j]
        if len(self.ped_caps):
            d = np.concatenate([d, self._pedestal_distances(A, B, r)])
        return A, B, d, ''

    def _membership(self):
        """Arms come and go (the GUI starts them one by one): an arm is guarded while its
        robot_description has a publisher. A change rebuilds the capsules and re-runs the
        start-up (a short hold while nothing moves)."""
        for arm in list(self.descriptions):
            if self.count_publishers(f'/{arm}/robot_description') == 0:
                del self.descriptions[arm]
        live = set(self.descriptions)
        if self.pairs is not None and live != self.built_arms:
            self.get_logger().warn(f'arms changed {sorted(self.built_arms)} -> {sorted(live)}: '
                                   'rebuilding')
            self.pairs = None
            self.started = False
            self.recovery = None
            self.t0 = time.monotonic() - self.wait_s   # no extra wait on a rebuild

    def _tick(self):
        if time.monotonic() - self.last_membership > 1.0:
            self.last_membership = time.monotonic()
            self._membership()
        if self.pairs is None:
            if (time.monotonic() - self.t0 < self.wait_s and
                    len(self.descriptions) < len(self.arms)):
                return
            if not self.descriptions:
                self.tf_fault = 'no arm description'
                self._update_hold()
                return
            missing = [fr for sp in self.sphere_cfg for fr in sp['between']
                       if not self.tf.can_transform(self.world, fr, Time())]
            if missing:
                # Never build without the pedestal: hold (starting) until its frames arrive.
                self.tf_fault = f'no tf for the pedestal spheres ({", ".join(sorted(set(missing)))})'
                self._update_hold()
                return
            self._build()
            self.built_arms = set(self.descriptions)
        A, B, d, why = self._distances()
        self.tf_fault = why
        if A is None:
            self._update_hold()
            self.state_pub.publish(String(data=json.dumps({
                'hold': bool(self.hold), 'reason': self.reason,
                'recovery': self.recovery is not None, 'closest': []})))
            return
        if not self.started:
            self._exclude_start_contacts(d)
            self.started = True
            A, B, d, _ = self._distances()
        close = np.nonzero(d < self.stop)[0]
        if self.recovery is not None:
            worse = [k for k in close if k not in self.recovery or
                     d[k] < self.recovery[k] - self.slack]
            if worse:
                self.recovery = None
                self.collision = 'closer during recovery: ' + self._name(worse[0], d)
            elif np.all(d > self.stop + self.clear_margin):
                self.recovery = None
                self.get_logger().info('recovery ended: every pair clear')
        elif not self.collision and len(close):
            self.collision = self._name(close[np.argmin(d[close])], d)
        self._update_hold()
        order = np.argsort(d)[:5]
        r = np.array([c.r for c in self.capsules])
        self.state_pub.publish(String(data=json.dumps({
            'hold': bool(self.hold), 'reason': self.reason, 'recovery': self.recovery is not None,
            'closest': [{'pair': self._name(k), 'mm': round(1e3 * float(d[k]), 1)} for k in order]})))
        if time.monotonic() - self.last_marker > 0.1:
            self.last_marker = time.monotonic()
            self._markers(A, B, r, d)

    def _name(self, k, d=None):
        if k >= len(self.pairs):
            ci = self.capsules[self.ped_caps[k - len(self.pairs)]]
            s = f'{ci.arm}/{ci.name} - pedestal'
            return s + (f' {1e3 * d[k]:.0f} mm' if d is not None else '')
        ci, cj = self.capsules[self.pairs[k, 0]], self.capsules[self.pairs[k, 1]]
        s = f'{ci.arm}/{ci.name} - {cj.arm}/{cj.name}'
        return s + (f' {1e3 * d[k]:.0f} mm' if d is not None else '')

    def _update_hold(self):
        if not self.started:
            why = 'starting' + (f': {self.tf_fault}' if self.tf_fault else '')
        else:
            why = self.collision or self.tf_fault
        on = bool(why)
        kind = re.sub(r'[-+]?\d[\d.]*', '#', why)   # the reason without its numbers
        changed_kind = kind != re.sub(r'[-+]?\d[\d.]*', '#', self.reason)
        if on and not self.hold:
            self.get_logger().error(f'HOLD: {why}')
        elif on and changed_kind:
            self.get_logger().warn(f'still holding: {why}')
        elif not on and self.hold:
            self.get_logger().info('hold cleared')
        publish = on != self.hold
        self.hold, self.reason = on, why
        if publish:
            self.hold_pub.publish(Bool(data=on))

    def _release(self, req, res):
        if not self.collision:
            res.success = not self.hold
            res.message = ('not holding' if not self.hold else
                           f'nothing to release: {self.reason} (clears by itself)')
            return res
        A, B, d, why = self._distances()
        if A is None:
            res.success, res.message = False, f'cannot release: {why}'
            return res
        self.recovery = {int(k): float(d[k]) for k in np.nonzero(d < self.stop)[0]}
        self.collision = ''
        self._update_hold()
        res.success = True
        res.message = (f'released into recovery ({len(self.recovery)} pairs inside '
                       f'{1e3 * self.stop:.0f} mm may only separate)')
        self.get_logger().warn(res.message)
        return res

    def _markers(self, A, B, r, d):
        level = np.zeros(len(self.capsules))   # 0 green, 1 yellow, 2 red
        for k, (i, j) in enumerate(self.pairs):
            lv = 2 if d[k] < self.stop else (1 if d[k] < self.warn else 0)
            level[i] = max(level[i], lv)
            level[j] = max(level[j], lv)
        for n, i in enumerate(self.ped_caps):
            dk = d[len(self.pairs) + n]
            level[i] = max(level[i], 2 if dk < self.stop else (1 if dk < self.warn else 0))
        colors = [(0.2, 0.8, 0.2, 0.25), (1.0, 0.8, 0.0, 0.5), (1.0, 0.1, 0.1, 0.7)]
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for k, c in enumerate(self.capsules):
            col = colors[int(level[k])]
            pts = [A[k], B[k]]
            line = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns='axis', id=k)
            line.header.frame_id, line.header.stamp = self.world, stamp
            line.scale.x = 2 * c.r
            line.points = [Point(x=float(q[0]), y=float(q[1]), z=float(q[2])) for q in pts]
            line.color.r, line.color.g, line.color.b, line.color.a = col
            line.pose.orientation.w = 1.0
            ma.markers.append(line)
            for e, q in enumerate(pts):
                s = Marker(type=Marker.SPHERE, action=Marker.ADD, ns=f'end{e}', id=k)
                s.header.frame_id, s.header.stamp = self.world, stamp
                s.pose.position.x, s.pose.position.y, s.pose.position.z = map(float, q)
                s.pose.orientation.w = 1.0
                s.scale.x = s.scale.y = s.scale.z = 2 * c.r
                s.color.r, s.color.g, s.color.b, s.color.a = col
                ma.markers.append(s)
        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    n = Guard()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
