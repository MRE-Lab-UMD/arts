# da Vinci X/Xi instrument adapter — design and manufacturing package

**Role: hardware design source (first-party, published with ARTS).** What to make and buy for
one adapter. Cost per adapter and the system total: `docs/bom/bom.csv`. Placement on the arm
and the measured shaft line: `arts_description/scripts/import_tool_step.py adapter`
(88.388 mm from the flange face, along tool0 +y).

The adapter bolts to a UR flange (ISO 9409-1-50-4-M6) and drives a da Vinci X/Xi instrument's
input discs with five Dynamixel XC330 servos through GT2 belts and machined stainless shafts.

Per adapter: 11 printed part types (16 parts), 10 machined shafts, and bought parts (BOM
below). The stapler and knob-securing parts are optional.

## Bought parts (one adapter; designer's BOM, 2026-11-02)

| Item | Qty | Vendor | Part number |
|---|---|---|---|
| Dynamixel XC330-M288-T servo | 5 | Robotis | XC330-M288-T |
| U2D2 Power Hub Board set | 1 | Robotis | U2D2 PHB |
| U2D2 (USB to Dynamixel) | 1 | Robotis | U2D2 |
| GT2 timing belt, high torque, 106 mm | 4 | MiSUMi | GBN1062GT-40 |
| GT2 timing belt, high torque, 128 mm (middle shaft) | 1 | MiSUMi | GBN1282GT-40 |
| GT2 timing belt pulley, high torque | 10 | MiSUMi | GPA22GT2040-B-H4 |
| Micro spring plunger (inner shafts; 1 spare) | 5 | MiSUMi | MPFS3-3 |
| Ball bearing, 7 mm ID (spring-plunger seats) | 5 | MiSUMi | SUTB117AZZ |
| Ball bearing, 4 mm ID (shafts) | 10 | MiSUMi | SUTB84A1ZZ |
| 16 AWG male-female power extension cord (up the arm) | 1 | any | — |
| M3 socket screw assortment (6–30 mm) | 1 | any | — |
| 5 V 10 A power supply for the U2D2 power hub | 1 | any | — |
| M3 heat-set inserts, 3.4 mm, pack of 50 | 1 | McMaster-Carr | — |
| 4-40 set screws, pack of 10 (pulleys, motor adjusters) | 2 | McMaster-Carr | — |
| M6 x 16 mm socket head screws, pack of 25 (adapter to UR flange) | 1 | McMaster-Carr | — |
| Optional: M6 heat-set inserts, pack of 25 (star knobs) | 1 | McMaster-Carr | — |
| Optional: star knobs GN 6336.12-32-M6-25-15 | 3 | JW Winco | GN 6336.12-32-M6-25-15 |

## 3D-printed parts (`printed/`)

| File | Qty | Solid volume (cm3) |
|---|---|---|
| 02_UR_Connector_top | 1 | 144.0 |
| 03_UR5_Connector_bottom | 1 | 94.5 |
| 04_Disk_Plate_0.5 | 1 | 16.6 |
| 05_Engagement_Disks | 5 | 2.0 each |
| 06_Snap_Clips | 2 | 0.7 each |
| 13_8mm_Clip | 1 | 6.2 |
| 16_U2D2_Board_Holder | 1 | 28.5 |
| 17_Motor_Holder_Outer | 4 | 7.5 each |
| 18_Motor_Holder_Inner | 1 | 8.0 |
| 22_Motor_Twist_Top | 5 | 0.7 each |
| 8mm_Tool_Support_Channel | 1 | 47.2 |

About 390 cm3 of solid volume per adapter (under 0.5 kg of filament at full infill).
Optional: `OPTIONAL_03_Top_Securing_Forceps_Scissors` (knob-secured top, with the M6 inserts
and star knobs), and for the stapler `OPTIONAL_03_Top_Securing_stapler`,
`OPTIONAL_13_Rod_Support_Base_Clip_stapler`, `OPTIONAL_14mm_Tool_Support_Channel_Stapler`.

## CNC-machined shafts (`machined/`, drawings in `drawings/`)

Five disc drives per adapter, each an inner shaft inside an outer shaft: **5x inner + 5x outer
per adapter.**

| Part | Model | Envelope | Volume |
|---|---|---|---|
| Inner shaft | `machined/inner_shaft.step`, `drawings/inner_shaft_drawing_metric.pdf` | 38.45 x 7.70 x 7.70 mm | 780.81 mm3 |
| Outer shaft | `machined/outer_shaft.step`, `drawings/outer_shaft_drawing_metric.pdf` | 37.00 x 16.00 x 16.00 mm | 960.35 mm3 |

Manufacturing notes, as quoted (online CNC service, 2026-08-21):

- Process: CNC machining, no subprocess preference.
- Material: stainless steel 316/316L. Finish: standard (as machined).
- Threads and tapped holes: none. Inserts: none.
- Precision tolerance: tighter than ±0.0005 in (±0.013 mm) at 5 locations on the inner
  shaft, 2 on the outer (the h6/H7 fits and runouts on the drawings: inner Ø7 h6 and Ø4 h6
  bearing seats, Ø3 H7 bore, 0.025 runout to datum A; outer Ø4 h6, 0.025 runout).
- Surface roughness: 125 µin / 3.2 µm Ra at its smallest.
- Inspection: standard. Upload the drawings with the models: the service does not read
  tolerances from the 3D model.
- Price (Xometry instant quote, made internationally, 15 business days): inner 58.91 each at
  qty 1, 35.35 each at qty 15; outer 62.88 at qty 1, 37.73 at qty 15. Shipping and duties extra.

`adapter_assembly.step` (AP214, millimetres) is the whole adapter, with motors, bearings,
belts and fasteners; part names carry the vendor part numbers. The drawings are rasters with
the title block cleared; every dimension and tolerance is kept.
