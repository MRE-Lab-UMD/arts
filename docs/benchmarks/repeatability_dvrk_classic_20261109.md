# Pose repeatability by optical tracker, dVRK Classic PSM, 2026-11-09

**Layer 4 (measured numbers).** Companion of `repeatability_psm_left_20261107.md` (ARTS, same
protocol, tracker and marker). Measured by the dVRK-side collaborators; analysed here.

## Data and assumptions

- Received as `cluster_1.npy` .. `cluster_5.npy`: one file per pose, 30 rows each (one per
  cycle, assumed in cycle order), columns x, y, z (metres) and a unit quaternion (assumed x, y,
  z, w, the ROS order). Machine-local copy: `~/Downloads/atracsys_poses_dvrk_classic/`.
- Statistics as in the ARTS report: l = distance of each attained position from its pose's
  barycentre, RP = mean(l) + 3 std(l) (n - 1); RPa/b/c = 3 std of the rotation-vector
  components relative to the mean rotation. Pose separations 18-71 mm.
- Collaborator (2026-11-10): the same marker, with its spheres unscrewed and remounted; their
  pooled figure 0.06 +- 0.17 mm matches the pooled row below; the single large value (cycle 22)
  is considered a legitimate measurement, not a tracking error, so it stays in.

## Results

| Pose | n | RP mm | mean l | std l | RMS l | max l (mm) | RPa / RPb / RPc deg |
|---|---|---|---|---|---|---|---|
| P1 | 30 | 0.100 | 0.036 | 0.021 | 0.042 | 0.094 | 0.108 / 0.036 / 0.043 |
| P2 | 30 | 0.351 | 0.062 | 0.096 | 0.113 | 0.554 | 0.332 / 0.339 / 0.194 |
| P3 | 30 | 0.106 | 0.044 | 0.021 | 0.048 | 0.081 | 0.057 / 0.041 / 0.097 |
| P4 | 30 | 0.090 | 0.031 | 0.020 | 0.037 | 0.071 | 0.067 / 0.049 / 0.056 |
| P5 | 30 | 1.239 | 0.143 | 0.365 | 0.387 | 2.075 | 0.657 / 0.479 / 0.482 |
| pooled | 150 | | 0.0633 | 0.1724 | 0.1831 | 2.0750 | |

**Cycle 22 is an outlier at both P2 (0.554 mm) and P5 (2.075 mm)**, the only samples above
0.12 mm. Without cycle 22 (29 samples): RP P2 0.110 mm, P5 0.072 mm, so 0.07-0.11 mm at every
pose. Reported with it; the exclusion is shown only to locate the effect.

## Side by side (per-pose RP, mm)

| | min | median | max |
|---|---|---|---|
| ARTS UR5e + ProGrasp (2026-11-07) | 0.139 | 0.203 | 0.252 |
| dVRK Classic, all cycles | 0.090 | 0.106 | 1.239 |
| dVRK Classic, without cycle 22 | 0.072 | 0.100 | 0.110 |
