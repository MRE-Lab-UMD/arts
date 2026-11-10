#!/usr/bin/env python3
"""A flange-mounted tool's assembly STEP -> a viewing mesh in the <arm>_tool0 frame.

Two first-party tools are bolted to UR flanges: the da Vinci X/Xi instrument adapter (tool
arms) and the stereo laparoscope (camera arm). Their design packages are in the repo under
hardware/. This script turns either assembly into one binary STL placed in the UR
flange frame, and measures from the CAD where the tool's axis (the instrument shaft line or
the endoscope axis) sits relative to the flange, which is what the controller's tool-tip
preset depends on:

    $ARTS_LOCAL/venvs/pedestal_import/bin/python -I import_tool_step.py adapter \\
        ~/arts/hardware/tool_adapter/adapter_assembly.step
    $ARTS_LOCAL/venvs/pedestal_import/bin/python -I import_tool_step.py laparoscope \\
        ~/arts/hardware/laparoscope/laparoscope_assembly.step

(the same pinned venv as import_pedestal_step.py). Each tool's part names and radii are in
TOOLS below; every one can be overridden on the command line.

Frames, all read from the CAD's ISO 9409-1-50-4-M6 interface (4 x M6 on a 50 mm circle and
one 6 mm dowel at 25 mm) rather than assumed:
  - the flange axis is the centre of the four M6 clearance holes (--bolt-radius) of the part
    that bolts to the UR (--flange-part);
  - the flange FACE is the end of those holes away from their counterbores (--head-radius,
    where the screw heads sit); tool0 +z points from the face into the tool;
  - tool0 -y points from the flange axis toward the dowel (--dowel-radius), and x = y x z.
    Checked on the real arms 2026-11-02 for the adapter (PI, RViz over the read-only
    bringup); the same rule places the laparoscope.
The report gives the tool axis (--axis-part, --axis-radius): its distance from the flange
face along tool0 z, its lateral offset from the flange axis, its angle to the flange plane
and its direction in tool0.

Fasteners and inserts (--exclude) are left out of the mesh to keep it small.
Output (--out-dir, default the machine-local catalog's <tool>/2_mesh/visual, where cart_tf
looks): <tool>.stl and <tool>_import.yaml. The mesh is generated, never committed.
"""
import argparse
import math
import os
import re
import sys
import time

import numpy as np
import yaml
from OCP.BRep import BRep_Builder
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.gp import gp_Ax3, gp_Dir, gp_Pnt, gp_Trsf
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.XCAFDoc import XCAFDoc_DocumentTool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from import_pedestal_step import Walker  # noqa: E402

# Part names and radii (mm) in each tool's assembly STEP.
TOOLS = {
    'adapter': {
        'catalog': '3_tool_adapter',
        'flange_part': '02_UR_Connector_top', 'bolt_radius': 3.5, 'head_radius': 6.5,
        'dowel_radius': 3.02,
        'axis_part': '8mm_Tool_Support_Channel', 'axis_radius': 4.5,  # instrument channel bore
        'exclude': r'screws|heat_inserts|MPFS3-3',
    },
    'laparoscope': {
        'catalog': '5_laparoscope',
        'flange_part': 'Stereo_Lap_Cam_Mount', 'bolt_radius': 3.3, 'head_radius': 5.5,
        'dowel_radius': 3.0,
        'axis_part': 'Stereo_Lap_Cam_Scope', 'axis_radius': 6.0,  # the 12 mm endoscope tube
        'exclude': r'^(ER05|92125A|91292A|94459A|92196A)',
    },
}


def cylinders(shape, radius, tol=0.01):
    """(axis point, unit direction, face bbox) of every cylindrical face of that radius."""
    out = []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face_s(ex.Current())
        s = BRepAdaptor_Surface(face)
        if s.GetType() == GeomAbs_Cylinder and abs(s.Cylinder().Radius() - radius) < tol:
            ax = s.Cylinder().Axis()
            p, d = ax.Location(), ax.Direction()
            box = Bnd_Box()
            BRepBndLib.Add_s(face, box)
            out.append((np.array([p.X(), p.Y(), p.Z()]), np.array([d.X(), d.Y(), d.Z()]),
                        np.array(box.Get())))
        ex.Next()
    return out


def part(kept, name):
    hits = [s for path, s in kept if path.split('/')[-1] == name]
    if len(hits) != 1:
        sys.exit(f'expected exactly one {name!r} in the assembly, found {len(hits)}')
    return hits[0]


def span(faces, axis):
    """(min, max) of a set of faces' bounding boxes projected on an axis (0.01 mm rounded)."""
    vals = []
    for _, _, bb in faces:
        corners = np.array([[x, y, z] for x in bb[[0, 3]] for y in bb[[1, 4]] for z in bb[[2, 5]]])
        vals += list(corners @ axis)
    return round(min(vals), 2), round(max(vals), 2)


def flange_frame(shape, a):
    """origin, x, y, z of tool0 in CAD millimetres, from the ISO 9409 interface."""
    bolts = cylinders(shape, a['bolt_radius'])
    centres = {tuple(np.round(p - (p @ (d / np.linalg.norm(d))) * d / np.linalg.norm(d), 2))
               for p, d, _ in bolts}
    if len(centres) != 4:
        sys.exit(f'expected 4 M6 holes of r={a["bolt_radius"]} mm, found {len(centres)}')
    z_cad = bolts[0][1] / np.linalg.norm(bolts[0][1])
    centre = np.mean([np.array(c) for c in centres], axis=0)
    pcd = 2 * np.mean([np.linalg.norm(np.array(c) - centre) for c in centres])
    lo, hi = span(bolts, z_cad)
    head_lo, head_hi = span(cylinders(shape, a['head_radius']), z_cad)
    # The face is the hole end away from the counterbores; +z points from it toward them.
    if abs(head_lo - hi) < abs(head_hi - lo):
        face_h, z_tool = lo, z_cad
    else:
        face_h, z_tool = hi, -z_cad
    origin = centre + (face_h - centre @ z_cad) * z_cad
    dowel = cylinders(shape, a['dowel_radius'])
    if not dowel:
        sys.exit(f'no r={a["dowel_radius"]} mm dowel on {a["flange_part"]}')
    dp = dowel[0][0] - origin
    dp -= (dp @ z_tool) * z_tool
    y_tool = -dp / np.linalg.norm(dp)
    x_tool = np.cross(y_tool, z_tool)
    return origin, x_tool, y_tool, z_tool, pcd, float(np.linalg.norm(dp))


def main():
    local = os.environ.get('ARTS_LOCAL', os.path.expanduser('~/arts_local'))
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('tool', choices=sorted(TOOLS))
    ap.add_argument('step')
    ap.add_argument('--out-dir')
    for k, v in TOOLS['adapter'].items():
        if k != 'catalog':
            ap.add_argument('--' + k.replace('_', '-'), type=type(v), help=f'default: per tool')
    ap.add_argument('--deflection', type=float, default=0.5, help='mesh chord tolerance, mm')
    args = ap.parse_args()
    a = dict(TOOLS[args.tool])
    for k in a:
        if getattr(args, k, None) is not None:
            a[k] = getattr(args, k)
    out_dir = args.out_dir or os.path.join(local, 'description_catalog', 'parts', a['catalog'],
                                           '2_mesh', 'visual')

    t0 = time.time()
    doc = TDocStd_Document(TCollection_ExtendedString('step'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(args.step) != IFSelect_RetDone or not reader.Transfer(doc):
        sys.exit(f'cannot read {args.step}')
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    w = Walker(tool, re.compile(a['exclude']))
    for i in range(1, roots.Length() + 1):
        w.walk(roots.Value(i), TopLoc_Location(), [])
    print(f'read in {time.time() - t0:.0f} s: kept {len(w.kept)} shapes, left out '
          f'{len(w.removed)} fasteners', flush=True)

    origin, x_tool, y_tool, z_tool, pcd, dowel_r = flange_frame(part(w.kept, a['flange_part']), a)

    # Tool axis: the closest point of its line to the flange axis.
    ax = cylinders(part(w.kept, a['axis_part']), a['axis_radius'])
    if not ax:
        sys.exit(f'no r={a["axis_radius"]} mm cylinder on {a["axis_part"]}')
    q, d = ax[0][0], ax[0][1] / np.linalg.norm(ax[0][1])
    w0 = q - origin
    aa, b, e, f = d @ d, d @ z_tool, d @ w0, z_tool @ w0
    den = aa - b * b
    s_ = (b * f - e) / den if abs(den) > 1e-12 else 0.0
    t_ = (aa * f - b * e) / den if abs(den) > 1e-12 else f
    on_axis_line, on_flange_axis = q + s_ * d, origin + t_ * z_tool
    body = Bnd_Box()
    BRepBndLib.Add_s(part(w.kept, a['axis_part']), body)
    body_c = np.array(body.Get()).reshape(2, 3).mean(axis=0)
    d_out = d if (body_c - origin) @ d > 0 else -d   # the direction the axis part extends
    in_tool0 = np.round([d_out @ x_tool, d_out @ y_tool, d_out @ z_tool], 4)

    frame = gp_Ax3(gp_Pnt(*origin), gp_Dir(*z_tool), gp_Dir(*x_tool))
    to_tool = gp_Trsf()
    to_tool.SetTransformation(frame)
    scale = gp_Trsf()
    scale.SetScale(gp_Pnt(0, 0, 0), 0.001)
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)
    for _, s in w.kept:
        builder.Add(comp, s)
    shape = BRepBuilderAPI_Transform(comp, scale.Multiplied(to_tool), True).Shape()

    os.makedirs(out_dir, exist_ok=True)
    stl = os.path.join(out_dir, f'{args.tool}.stl')
    BRepMesh_IncrementalMesh(shape, args.deflection * 0.001, False, 0.5, True)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(shape, stl):
        sys.exit(f'cannot write {stl}')
    # Extent from the mesh itself (a STEP solid can carry an infinite bounding box).
    raw = open(stl, 'rb').read()
    pts = np.frombuffer(raw[84:], dtype=np.dtype(
        [('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))['v'].reshape(-1, 3)
    report = {
        'tool': args.tool,
        'source': os.path.basename(args.step),
        'left_out': sorted({r.split('/')[-1] for r in w.removed}),
        'kept_shapes': len(w.kept),
        'flange': {'part': a['flange_part'], 'bolt_circle_mm': round(float(pcd), 3),
                   'dowel_from_axis_mm': round(dowel_r, 3)},
        'tool_axis': {
            'part': f'{a["axis_part"]} (r={a["axis_radius"]} mm)',
            'from_flange_face_along_tool0_z_mm': round(float((on_axis_line - origin) @ z_tool), 3),
            'lateral_offset_from_flange_axis_mm':
                round(float(np.linalg.norm(on_axis_line - on_flange_axis)), 3),
            'angle_to_flange_plane_deg': round(math.degrees(math.asin(min(1.0, abs(b)))), 3),
            'direction_in_tool0': in_tool0.tolist(),
        },
        'extent_m': {'min': [round(float(v), 4) for v in pts.min(axis=0)],
                     'max': [round(float(v), 4) for v in pts.max(axis=0)]},
        'mesh': stl,
        'mesh_bytes': os.path.getsize(stl),
        'seconds': round(time.time() - t0),
    }
    with open(os.path.join(out_dir, f'{args.tool}_import.yaml'), 'w') as fh:
        yaml.safe_dump(report, fh, sort_keys=False)
    print(yaml.safe_dump(report, sort_keys=False))


if __name__ == '__main__':
    main()
