# Stereo laparoscope coupling — design package

**Role: hardware design source (first-party, published with ARTS).** What to make and buy to
turn a da Vinci Si stereo endoscope into two machine-vision camera views on the camera arm.
Prices and the system total: `docs/bom/bom.csv` (the optics, camera and fastener rows).
Placement on the arm: `arts_description/scripts/import_tool_step.py laparoscope` (endoscope
axis 42.13 mm from the flange face, along tool0 +y).

The coupling bolts to the UR flange (ISO 9409-1-50-4-M6), holds the endoscope, and folds each
of its two optical channels through a prism and a mirror into a C-mount lens and a FLIR
Blackfly S camera.

## Files

- `laparoscope_assembly.step` (AP214, millimetres): the whole assembly, with the vendor
  optics, the endoscope as a close-enough shape made by the collaborators (not an Intuitive
  file), and the printed parts. It was assembled from slightly older part files than
  `printed/`: the printed STLs are the current parts (checked 2026-11-02: UR mount and
  light-cable adapter identical; prism plates within ~2 %; the light-cable holders and the
  back prism mount are newer than the STEP). The camera body in the STEP is a BFS-U3-120S4;
  ARTS uses the BFS-U3-63S4C-C, same case.
- `printed/` — 9 parts: UR mount, prism mount (top, bottom, back, left and right plates),
  light-cable holders (left, right), light-cable mount adapter. About 390 cm3 of solid volume.

## Bought parts

| Item | Qty | Vendor | Part number |
|---|---|---|---|
| Right-angle kinematic mirror mount, 30 mm cage | 2 | Thorlabs | KCB1C |
| 1 in protected silver mirror | 2 | Thorlabs | PF10-03-P01 |
| 30 mm cage plate, SM1 and C-mount bores | 2 | Thorlabs | CP36 |
| C-mount to SM1 adapter | 2 | Thorlabs | SM1A9 |
| SM1 lens tube 0.30 in | 2 | Thorlabs | SM1L03 |
| Right-angle prism mirror, 25 mm | 2 | Thorlabs | MRA25-P01 |
| Cage rod 1/2 in, 6 mm | 14 | Thorlabs | ER05 |
| Male M30.5 to female C-mount adapter | 2 | Edmund Optics | 33-429 |
| 50 mm C Series fixed focal length lens | 2 | Edmund Optics | 59-873 |
| Blackfly S USB3 colour camera | 2 | Teledyne FLIR | BFS-U3-63S4C-C |
| M3 x 10 flat head screw, 18-8 | 8 | McMaster-Carr | 92125A130 |
| M3 x 55 socket head screw, 18-8 | 2 | McMaster-Carr | 91292A027 |
| M3 x 25 flat head screw, 90 deg, 18-8 | 2 | any | ISO 10642 |
| M3 x 30 socket head screw, 18-8 | 4 | McMaster-Carr | 91292A022 |
| M3 heat-set insert, brass, 5.7 mm | 14 | McMaster-Carr | 94459A140 |
| 8-32 x 5/8 in socket head screw, 18-8 | 4 | McMaster-Carr | 92196A196 |
| 1/4-20 x 7/16 in socket head screw, 18-8 | 2 | McMaster-Carr | 92196A536 |
| da Vinci Si stereo endoscope, 0 deg, 12 mm (used) | 1 | Intuitive Surgical | 370892 |
| da Vinci Si light guide, bottom panel removed (used) | 1 | Intuitive Surgical | 951026 |
| da Vinci Si camera head drape | 1 | Intuitive Surgical | 420273 |

The Intuitive parts are research surplus, used for research only, never clinically.
