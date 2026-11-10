"""The cart-level edges of the one tf tree (decisions 12, 13, 18): world -> everything placed.

Exactly one node publishes these. Each arm's own description is rooted at its <arm>_mount and
its robot_state_publisher publishes from there down; this node places the mounts, the
pedestal, the derived cart_center, and the console with the MTM bases. Numbers come from
arts_description/config/mount_transforms.yaml (CAD-nominal, placeholder today); measured
corrections (arts_calibration/config/<frame>_correction.yaml, decision 12) are applied on top
when they exist -- none do yet, so none are read.

It also publishes the pedestal for RViz on ~/pedestal (/cart_tf/pedestal,
visualization_msgs/MarkerArray, latched), in the `pedestal` frame: the Vention mesh when it has
been imported into the machine-local tree (arts_description/scripts/import_pedestal_step.py;
Vention's geometry is never committed), otherwise the placeholder box from
mount_transforms.yaml. The mesh's own placement relative to `pedestal` is
mount_transforms.yaml `pedestal_mesh` (identity when absent). When the importer's
ur5e_bases_reference.stl is there too, it is drawn translucent red in the same placement: the
UR5e bases where Vention's assembly put them, to check the mount poses against the arms.

On the same topic, for each tool arm: the instrument adapter mesh in `<arm>_tool0`
(frame-locked, so it follows the arm) when it has been imported
(arts_description/scripts/import_tool_step.py adapter), and the nominal instrument shaft as a
translucent 8 mm cylinder along tool0 +y, 88.4 mm out (the adapter CAD), from the adapter to
the wrist at 609 mm (the `da_vinci_x_wrist` tip preset). Viewing only: the controller's own
tip is the `<arm>_tool_tip` frame. The stereo laparoscope mesh hangs on `ecm_tool0` the same
way (import_tool_step.py laparoscope).
"""
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

ARMS = ('psm_left', 'psm_right', 'ecm')
TOOL_ARMS = ('psm_left', 'psm_right')
# Nominal instrument shaft in tool0 (m): along +y, SHAFT_Z out from the flange face (the
# adapter CAD's 88.388 mm), from the adapter's far end to the wrist (tool_tips.yaml
# da_vinci_x_wrist, 609 mm). 8 mm instruments.
SHAFT_Z = 0.0884
SHAFT_FROM_Y, SHAFT_TO_Y = 0.186, 0.609
SHAFT_DIAMETER = 0.008
# The ECM's nominal stereo VIEW frame, `ecm_stereo_optical_frame` (decision 18 names): at the scope tip from the laparoscope
# CAD import (axis 42.13 mm out from the flange face, along ecm_tool0 +y, tip at 0.5666 m), in
# the ROS OPTICAL convention: z forward out of the scope into the scene (point-cloud depth is
# +z), x = image right, y = image down. Nominally x = tool0 x, y = -tool0 z, z = tool0 y, a
# -90 deg turn about x; `view_roll_deg` turns it about its own z for the camera mounting and the
# display's rotation (the first teleop sign check). The teleop follower's screen frame is this
# turned 180 deg about x (decision 17, 2026-11-03 amendment). NOT a calibrated frame: after the
# stereo + one hand-eye calibration (decision 12) it becomes the derived centre of the rectified
# pair, beside ecm_stereo_{left,right}_optical_frame; point clouds belong to the left one.
SCOPE_Z, SCOPE_TIP_Y = 0.04213, 0.5666


def _quat(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r / 2), np.sin(r / 2), np.cos(p / 2), np.sin(p / 2), np.cos(y / 2), np.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def _edge(stamp, parent, child, xyz, rpy):
    t = TransformStamped()
    t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
    (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
     t.transform.rotation.w) = map(float, _quat(rpy))
    return t


def main():
    rclpy.init()
    node = Node('cart_tf')
    with open(os.path.join(get_package_share_directory('arts_description'), 'config',
                           'mount_transforms.yaml')) as f:
        cfg = yaml.safe_load(f)

    def v(s):
        return [float(x) for x in s.split()]

    now = node.get_clock().now().to_msg()
    edges = [_edge(now, 'world', 'pedestal', (0, 0, 0), (0, 0, 0))]
    mounts = {a: cfg['mounts'][a] for a in ARMS}
    for a, m in mounts.items():
        edges.append(_edge(now, 'world', f'{a}_mount', v(m['xyz']), v(m['rpy'])))
    center = np.mean([v(m['xyz']) for m in mounts.values()], axis=0)
    edges.append(_edge(now, 'world', 'cart_center', center, (0, 0, 0)))
    edges.append(_edge(now, 'world', 'console', v(cfg['console']['xyz']), v(cfg['console']['rpy'])))
    for mtm, m in cfg['mtm'].items():
        edges.append(_edge(now, 'console', f'{mtm}_base', v(m['xyz']), v(m['rpy'])))
    view_roll = np.radians(node.declare_parameter('view_roll_deg', 0.0).value)
    # Rx(-90) Rz(roll) = Ry(roll) Rx(-90): rpy (-90 deg, roll, 0) in ROS's fixed-axis order.
    edges.append(_edge(now, 'ecm_tool0', 'ecm_stereo_optical_frame', (0.0, SCOPE_TIP_Y, SCOPE_Z),
                       (-np.pi / 2, view_roll, 0.0)))
    StaticTransformBroadcaster(node).sendTransform(edges)

    local = os.environ.get('ARTS_LOCAL', os.path.expanduser('~/arts_local'))
    mesh = node.declare_parameter('pedestal_mesh', os.path.join(
        local, 'description_catalog', 'parts', '1_pedestal', '2_mesh', 'visual',
        'pedestal.stl')).value
    m = Marker()
    m.header.stamp, m.header.frame_id = now, 'pedestal'
    m.ns, m.id, m.action = 'pedestal', 0, Marker.ADD
    m.color.r = m.color.g = m.color.b = 0.62
    m.color.a = 1.0
    if mesh and os.path.isfile(mesh):
        place = cfg.get('pedestal_mesh', {'xyz': '0 0 0', 'rpy': '0 0 0'})
        m.type, m.mesh_resource = Marker.MESH_RESOURCE, 'file://' + mesh
        m.scale.x = m.scale.y = m.scale.z = 1.0
        xyz, rpy = v(place['xyz']), v(place['rpy'])
        node.get_logger().info(f'pedestal: Vention mesh {mesh}')
    else:
        box = cfg['pedestal']
        m.type = Marker.CUBE
        m.scale.x, m.scale.y, m.scale.z = v(box['size'])
        xyz, rpy = v(box['xyz']), v(box['rpy'])
        node.get_logger().info('pedestal: placeholder box (no imported mesh at '
                               f'{mesh}; see arts_description/scripts/import_pedestal_step.py)')
    m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
    (m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
     m.pose.orientation.w) = _quat(rpy)
    markers = MarkerArray(markers=[m])
    ref = os.path.join(os.path.dirname(mesh), 'ur5e_bases_reference.stl') if mesh else ''
    # The arm bases from the Vention assembly, half-transparent red: a check of the mount
    # transforms against the real arms (done 2026-11-02), off by default since 2026-11-07
    # (operators read the red as a collision warning).
    show_ref = node.declare_parameter('show_cad_bases', False).value
    if show_ref and m.type == Marker.MESH_RESOURCE and os.path.isfile(ref):
        r = Marker()
        r.header, r.pose, r.scale = m.header, m.pose, m.scale
        r.ns, r.id, r.action, r.type = 'pedestal', 1, Marker.ADD, Marker.MESH_RESOURCE
        r.mesh_resource = 'file://' + ref
        r.color.r, r.color.g, r.color.b, r.color.a = 0.9, 0.2, 0.2, 0.5
        markers.markers.append(r)
    adapter = node.declare_parameter('adapter_mesh', os.path.join(
        local, 'description_catalog', 'parts', '3_tool_adapter', '2_mesh', 'visual',
        'adapter.stl')).value
    have_adapter = bool(adapter) and os.path.isfile(adapter)
    scope = node.declare_parameter('laparoscope_mesh', os.path.join(
        local, 'description_catalog', 'parts', '5_laparoscope', '2_mesh', 'visual',
        'laparoscope.stl')).value
    if scope and os.path.isfile(scope):
        lap = Marker()
        lap.header.stamp, lap.header.frame_id = now, 'ecm_tool0'
        lap.ns, lap.id, lap.action, lap.type = 'laparoscope', 0, Marker.ADD, Marker.MESH_RESOURCE
        lap.mesh_resource = 'file://' + scope
        lap.frame_locked = True
        lap.scale.x = lap.scale.y = lap.scale.z = 1.0
        lap.pose.orientation.w = 1.0
        lap.color.r, lap.color.g, lap.color.b, lap.color.a = 0.80, 0.80, 0.85, 1.0
        markers.markers.append(lap)
        node.get_logger().info(f'laparoscope: mesh {scope} on ecm_tool0')
    else:
        node.get_logger().info('laparoscope: no imported mesh '
                               '(arts_description/scripts/import_tool_step.py laparoscope)')
    for k, arm in enumerate(TOOL_ARMS):
        if have_adapter:
            a = Marker()
            a.header.stamp, a.header.frame_id = now, f'{arm}_tool0'
            a.ns, a.id, a.action, a.type = 'adapter', k, Marker.ADD, Marker.MESH_RESOURCE
            a.mesh_resource = 'file://' + adapter
            a.frame_locked = True
            a.scale.x = a.scale.y = a.scale.z = 1.0
            a.pose.orientation.w = 1.0
            a.color.r, a.color.g, a.color.b, a.color.a = 0.85, 0.85, 0.80, 1.0
            markers.markers.append(a)
        shaft = Marker()
        shaft.header.stamp, shaft.header.frame_id = now, f'{arm}_tool0'
        shaft.ns, shaft.id, shaft.action, shaft.type = 'instrument_shaft', k, Marker.ADD, \
            Marker.CYLINDER
        shaft.frame_locked = True
        y0, y1 = SHAFT_FROM_Y, SHAFT_TO_Y
        shaft.pose.position.y, shaft.pose.position.z = (y0 + y1) / 2, SHAFT_Z
        (shaft.pose.orientation.x, shaft.pose.orientation.y, shaft.pose.orientation.z,
         shaft.pose.orientation.w) = _quat((-np.pi / 2, 0.0, 0.0))  # cylinder z -> tool0 y
        shaft.scale.x = shaft.scale.y = SHAFT_DIAMETER
        shaft.scale.z = y1 - y0
        shaft.color.r, shaft.color.g, shaft.color.b, shaft.color.a = 0.3, 0.5, 0.9, 0.6
        markers.markers.append(shaft)
        # Labels for the flange and the controller's tip only (RViz's TF display labels every
        # frame or none).
        for j, frame in enumerate((f'{arm}_tool0', f'{arm}_tool_tip')):
            t = Marker()
            t.header.stamp, t.header.frame_id = now, frame
            t.ns, t.id, t.action, t.type = 'tip_labels', 2 * k + j, Marker.ADD, \
                Marker.TEXT_VIEW_FACING
            t.frame_locked = True
            t.text = frame
            t.pose.position.z = 0.03
            t.pose.orientation.w = 1.0
            t.scale.z = 0.03
            t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 1.0, 0.3, 1.0
            markers.markers.append(t)
    view_label = Marker()
    view_label.header.stamp, view_label.header.frame_id = now, 'ecm_stereo_optical_frame'
    view_label.ns, view_label.id, view_label.action, view_label.type = \
        'view_label', 0, Marker.ADD, Marker.TEXT_VIEW_FACING
    view_label.frame_locked = True
    view_label.text = 'ecm_stereo_optical_frame'
    view_label.pose.position.y = -0.03
    view_label.pose.orientation.w = 1.0
    view_label.scale.z = 0.03
    view_label.color.r, view_label.color.g, view_label.color.b, view_label.color.a = \
        1.0, 1.0, 0.3, 1.0
    markers.markers.append(view_label)
    node.get_logger().info('adapter: ' + (f'mesh {adapter}' if have_adapter else
                           'no imported mesh (arts_description/scripts/import_tool_step.py adapter)'
                           ) + '; nominal instrument shaft drawn on ' + ', '.join(TOOL_ARMS))
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(MarkerArray, '~/pedestal', latched)
    pub.publish(markers)
    node.get_logger().info('cart edges: ' + ', '.join(f'{e.header.frame_id}->{e.child_frame_id}'
                                                     for e in edges))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
