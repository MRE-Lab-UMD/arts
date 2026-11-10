#!/usr/bin/env python3
"""Vention pedestal STEP -> a viewing mesh in the machine-local tree (never committed).

Vention's terms forbid redistributing its part geometry, so ARTS ships this script, not the
mesh. Each user downloads the assembly from Vention themselves:

  1. Open the stand's public Vention design page (its link is in docs/bom/; withheld from the
     review capsule) -> "Open in 3D" (a free Vention account is needed) -> Downloads / 3D
     Models -> "Download Assembly" (VentionAssembly_<design>_v<N>.STEP).
  2. Copy it to $ARTS_LOCAL/description_catalog/parts/1_pedestal/0_incoming/.
  3. In the pinned venv (requirements.txt beside this script):
       python3 -m venv $ARTS_LOCAL/venvs/pedestal_import
       $ARTS_LOCAL/venvs/pedestal_import/bin/pip install -r requirements.txt
       $ARTS_LOCAL/venvs/pedestal_import/bin/python -I import_pedestal_step.py \\
           $ARTS_LOCAL/description_catalog/parts/1_pedestal/0_incoming/VentionAssembly_<design>_v<N>.STEP
  4. The script ends with the mount check (below); it must print `mount check PASSED`.
  5. cart_tf finds the mesh and publishes it for RViz (the arts.rviz "Pedestal" display).
The same steps, with the adapter and laparoscope imports, are in the README ("CAD meshes").

The assembly also holds three UR5e arms and three UR control boxes. The ARMS are removed:
every component whose STEP product name matches --exclude (default `^ur5e_`). The arms come
from the pinned ur_description, and another user may have other arms. The control boxes stay.

Frames: the STEP is millimetres, Y up. The mesh is written in metres, z up (REP-103): +90 deg
about x, then shifted so the footprint is centred on the origin and the lowest point is at
z = 0. That is a VIEWING placement, recorded in the report. The script also reports where the
assembly places each UR5e base (`ur5e_base_assy`), in the same frame: arm mount poses read
from Vention's own assembly, candidates for arts_description/config/mount_transforms.yaml
once the UR CAD's base origin is confirmed to be ur_description's base_link.

Mount check: the report's base poses are turned into world exactly as cart_tf places the mesh
(mount_transforms.yaml `pedestal_mesh`) and each is compared with the committed arm mount
(`mounts`, which carries the 180 deg base-yaw correction). Every committed mount must match one
assembly base within --mount-tol-mm / --mount-tol-deg, else the script exits 2: the download is
not the assembly ARTS's mounts were read from, and RViz would show the arms off the pedestal.

Output (--out-dir, default the catalog's 2_mesh/visual): pedestal.stl (binary),
ur5e_bases_reference.stl (the removed UR5e bases, same frame, for checking the mount poses
against ur_description) and pedestal_import.yaml (what was kept and removed, the transform,
the UR base poses).
"""
import argparse
import math
import os
import re
import sys
import time

import yaml
from OCP.BRep import BRep_Builder
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS_Compound
from OCP.XCAFDoc import XCAFDoc_DocumentTool


def label_name(label):
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ''


class Walker:
    """Depth-first over the XCAF assembly, composing locations down to the leaf shapes."""

    def __init__(self, tool, exclude):
        self.tool = tool
        self.exclude = exclude
        self.kept = []      # (path, located shape)
        self.removed = []   # paths
        self.bases = []     # TopLoc_Location of each ur5e_base_assy
        self.base_shapes = []  # those base assemblies, placed, for the reference mesh

    def walk(self, label, loc, path):
        tool = self.tool
        if tool.IsReference_s(label):
            ref = TDF_Label()
            tool.GetReferredShape_s(label, ref)
            self.walk(ref, loc.Multiplied(tool.GetLocation_s(label)), path)
            return
        name = label_name(label)
        if self.exclude.search(name):
            self.removed.append('/'.join(path + [name]))
            if name.startswith('ur5e_base_assy'):
                self.bases.append(loc)
                self.base_shapes.append(tool.GetShape_s(label).Moved(loc))
            return
        if tool.IsAssembly_s(label):
            comps = TDF_LabelSequence()
            tool.GetComponents_s(label, comps)
            for i in range(1, comps.Length() + 1):
                self.walk(comps.Value(i), loc, path + [name])
            return
        shape = tool.GetShape_s(label)
        if not shape.IsNull():
            self.kept.append(('/'.join(path + [name]), shape.Moved(loc)))


def pose(t):
    """gp_Trsf -> xyz (m, already scaled) and fixed-axis rpy (rad)."""
    v = t.TranslationPart()
    q = t.GetRotation()
    x, y, z, w = q.X(), q.Y(), q.Z(), q.W()
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return ([round(v.X(), 4), round(v.Y(), 4), round(v.Z(), 4)],
            [round(roll, 4), round(pitch, 4), round(yaw, 4)])


def _rot(rpy):
    """Fixed-axis roll, pitch, yaw -> 3x3 rotation (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _apply(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def _angle_deg(a, b):
    """Angle of the relative rotation a^T b."""
    tr = sum(a[k][i] * b[k][i] for i in range(3) for k in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1) / 2))))


def mount_check(bases, cfg_path, tol_mm, tol_deg):
    """Each committed mount vs the nearest assembly base, both in world. Returns (ok, lines)."""
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    vec = lambda s: [float(x) for x in s.split()]  # noqa: E731
    place = cfg.get('pedestal_mesh', {'xyz': '0 0 0', 'rpy': '0 0 0'})
    r_wm, t_wm = _rot(vec(place['rpy'])), vec(place['xyz'])
    flip = _rot([0.0, 0.0, math.pi])   # CAD base = UR controller "base" = base_link turned 180 deg
    world = []
    for b in bases:
        p = [a + c for a, c in zip(_apply(r_wm, b['xyz']), t_wm)]
        world.append((p, _mul(_mul(r_wm, _rot(b['rpy'])), flip)))
    ok, lines = True, []
    for arm, m in cfg['mounts'].items():
        pm, rm = vec(m['xyz']), _rot(vec(m['rpy']))
        if not world:
            ok = False
            lines.append(f'{arm}: no UR5e base in the assembly to compare with')
            continue
        d, ang = min(((math.dist(pm, p) * 1e3, _angle_deg(rm, r)) for p, r in world))
        good = d <= tol_mm and ang <= tol_deg
        ok &= good
        lines.append(f'{arm}: {"ok " if good else "OFF"} {d:.1f} mm, {ang:.2f} deg from the '
                     f'assembly base')
    return ok, lines


def main():
    local = os.environ.get('ARTS_LOCAL', os.path.expanduser('~/arts_local'))
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('step')
    ap.add_argument('--out-dir', default=os.path.join(
        local, 'description_catalog', 'parts', '1_pedestal', '2_mesh', 'visual'))
    ap.add_argument('--exclude', default=r'^ur5e_',
                    help='regex on STEP product names to drop (default: the UR5e arms)')
    # 2.5 mm is what ARTS's own RViz mesh was made with (2026-11-01, 79 MB); 1.0 gives 93 MB.
    ap.add_argument('--deflection', type=float, default=2.5, help='mesh chord tolerance, mm')
    ap.add_argument('--mounts', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'mount_transforms.yaml'),
        help='the committed mount file to check the assembly against')
    ap.add_argument('--mount-tol-mm', type=float, default=2.0)
    ap.add_argument('--mount-tol-deg', type=float, default=0.5)
    args = ap.parse_args()

    t0 = time.time()
    doc = TDocStd_Document(TCollection_ExtendedString('step'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(args.step) != IFSelect_RetDone or not reader.Transfer(doc):
        sys.exit(f'cannot read {args.step}')
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    w = Walker(tool, re.compile(args.exclude))
    for i in range(1, roots.Length() + 1):
        w.walk(roots.Value(i), TopLoc_Location(), [])
    print(f'read in {time.time() - t0:.0f} s: kept {len(w.kept)} shapes, removed '
          f'{len(w.removed)} components matching {args.exclude!r}, {len(w.bases)} UR5e bases',
          flush=True)
    if not w.kept:
        sys.exit('nothing left after the exclusion')
    if not w.removed:
        print(f'WARNING: nothing matched {args.exclude!r}; the arms may still be in the mesh')

    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)
    for _, s in w.kept:
        builder.Add(comp, s)

    # mm, Y up -> m, Z up; then centre the footprint and stand it on z = 0.
    rot = gp_Trsf()
    rot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), math.pi / 2)
    scale = gp_Trsf()
    scale.SetScale(gp_Pnt(0, 0, 0), 0.001)
    pre = scale.Multiplied(rot)
    shape = BRepBuilderAPI_Transform(comp, pre, True).Shape()
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    shift = gp_Trsf()
    shift.SetTranslation(gp_Vec(-(xmin + xmax) / 2, -(ymin + ymax) / 2, -zmin))
    shape = BRepBuilderAPI_Transform(shape, shift, True).Shape()
    full = shift.Multiplied(pre)

    os.makedirs(args.out_dir, exist_ok=True)

    def write_stl(s, path):
        BRepMesh_IncrementalMesh(s, args.deflection * 0.001, False, 0.5, True)
        writer = StlAPI_Writer()
        writer.ASCIIMode = False  # a property in OCP, not SetASCIIMode()
        if not writer.Write(s, path):
            sys.exit(f'cannot write {path}')

    stl = os.path.join(args.out_dir, 'pedestal.stl')
    write_stl(shape, stl)
    # The removed UR5e bases, in the same frame: laid over ur_description's arms in RViz they
    # show whether the UR CAD base origin and yaw are base_link's.
    ref = None
    if w.base_shapes:
        bcomp = TopoDS_Compound()
        builder.MakeCompound(bcomp)
        for s_ in w.base_shapes:
            builder.Add(bcomp, s_)
        ref = os.path.join(args.out_dir, 'ur5e_bases_reference.stl')
        write_stl(BRepBuilderAPI_Transform(bcomp, full, True).Shape(), ref)

    bases = []
    for loc in w.bases:
        xyz, rpy = pose(full.Multiplied(loc.Transformation()))
        bases.append({'xyz': xyz, 'rpy': rpy})
    report = {
        'source': os.path.basename(args.step),
        'exclude': args.exclude,
        'kept_shapes': len(w.kept),
        'removed_parts': sorted({r.split('/')[-1] for r in w.removed}),
        'transform': 'mm -> m; +90 deg about x (Y up -> Z up); footprint centred; min z = 0',
        'extent_m': [round(xmax - xmin, 4), round(ymax - ymin, 4), round(zmax - zmin, 4)],
        'ur5e_base_poses_in_mesh_frame': bases,
        'mesh': stl,
        'ur5e_bases_reference_mesh': ref,
        'mesh_bytes': os.path.getsize(stl),
        'seconds': round(time.time() - t0),
    }
    with open(os.path.join(args.out_dir, 'pedestal_import.yaml'), 'w') as f:
        yaml.safe_dump(report, f, sort_keys=False)
    print(yaml.safe_dump(report, sort_keys=False))
    ok, lines = mount_check(bases, args.mounts, args.mount_tol_mm, args.mount_tol_deg)
    print(f'mount check against {os.path.normpath(args.mounts)} '
          f'(tolerance {args.mount_tol_mm} mm, {args.mount_tol_deg} deg):')
    for line in lines:
        print('  ' + line)
    if not ok:
        print('MOUNT CHECK FAILED: this assembly does not match the committed arm mounts; the '
              'mesh was written but RViz will not line up with the arms.')
        sys.exit(2)
    print('mount check PASSED')


if __name__ == '__main__':
    main()
