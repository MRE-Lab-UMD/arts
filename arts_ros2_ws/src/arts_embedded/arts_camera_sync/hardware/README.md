# arts_camera_sync — hardware

KiCad source for the ARTS camera sync box lives here. Each revision is a **self-contained**
directory: open the `.kicad_pro` inside it and KiCad resolves project-local symbols,
footprints and 3-D models through `${KIPRJMOD}`, so a clean clone opens with no
user-specific library paths.

| Revision | Status |
|---|---|
| [`arts-camera-sync-v2/`](arts-camera-sync-v2/) | first published revision. Layout approved 2026-09-08; KiCad 9 source in-tree (`arts-camera-sync-v2.kicad_pro`); JLCPCB order queued (not yet purchased); **not fabricated**; no committed skew baseline |

## Why the design package exists before the schematic

Per AGENTS.md decision 14 the labour is split: the design **package** (connection table,
BOM, layout constraints, test procedure) is authored in this repo, the **drawing** is done
by someone with EDA experience, and the **review before ordering** happens here against the
package, pin by pin. DRC verifies geometry and says nothing about whether a footprint's
pin 1 is the pin the schematic meant — that error class produces a board which assembles
cleanly and mismeasures, which is the one outcome this box exists to prevent.

So `design_package/` is the specification, and it is the document a reviewer diffs the
finished schematic against. It is not a summary of the schematic written afterwards.

## Rules for this directory

**Toolchain is KiCad 9.** Its files are text, so diffs and reviews work, and it is what the
existing ESU-TAP board is drawn in — choosing anything else would mean maintaining two.

**Fabrication exports are committed only for a reviewed, fabricated revision**, under
`<revision>/production/`. Day-to-day Gerbers, BOMs and placement files are gitignored. An
export regenerated on every commit drifts out of step with the source silently, which is
worse than one being absent; but a reader without KiCad must still be able to order the
board that physically exists. Tying the export to a reviewed revision satisfies both.

**Nothing on the silkscreen identifies anybody.** No lab name, no personal name, no
institution, no repository URL. This tree becomes the anonymous review capsule, and a board
photograph in the paper is a tracked artifact like any other.

## Related documents

- Architecture and rationale: `AGENTS.md` decisions 14 and 15 (doc layer 1)
- Module doc: `docs/modules/pilot_flir_camera.md` Stage 5 (layer 2)
- Bench wiring guide for the breadboard rig this board replaces:
  `docs/modules/camera_sync_wiring.md` (layer 3)
- Measured baseline from the breadboard rig, which this board must beat:
  `docs/benchmarks/camera_sync_skew_breadboard_20260819.md` (layer 4)
- Firmware and the authoritative pin map: `../firmware/main/arts_sync_config.h`
