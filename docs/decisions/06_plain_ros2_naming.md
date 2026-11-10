# Decision 6 — Plain ROS 2 naming, no CRTK/dVRK vocabulary

> **Role: decision record.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

6. **Plain ROS 2 naming conventions. No CRTK / dVRK vocabulary.** ARTS is deliberately
   independent of dVRK (no dVRK support path for da Vinci Xi/X/SP/5 instruments; no reliance
   on company donations). ARTS defines its own instrument kinematics/coupling files.

## Amendment 2026-11-01 — role names exempted (decision 18)

The role names **psm, ecm and mtm** are exempted: namespaces `/psm_left`, `/psm_right`,
`/ecm`, `/mtm_left`, `/mtm_right`. Every surgical-robotics reader knows them, and the
alternatives ("tool arm left", "camera arm", "master device left") are longer and less
clear. Everything else in this decision stands. There are no CRTK command or state names
(`servo_cp`, `measured_js` …); topics use plain ROS 2 names under the suffix rule of
decision 18 (`_target` / `_command` / `_state`). No dVRK compatibility is implied, and ARTS
still owns its instrument kinematics and coupling files.
