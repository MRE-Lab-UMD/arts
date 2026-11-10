"""Pivot calibration (numpy only, no ROS), shared by pivot_monitor.py and rcm_drift_analyze.py.

A marker rigidly on the instrument shaft, the shaft turning about a fixed point: for every
sample i (marker pose R_i, p_i in the tracker frame) the same marker-frame point d lands on the
same tracker-frame point t:  R_i d + p_i = t. Stacked as [R_i | -I] [d; t] = -p_i and solved by
linear least squares (the standard pivot calibration; collaborator A's calibrator uses the same
formulation). The pivot error of sample i is |R_i d + p_i - t|.
"""
import numpy as np


def fit(positions, rotmats):
    """dict: d (marker frame), t (tracker frame), err (per sample), rmse, cond, range_deg; all
    lengths in the input unit. Needs >= 10 samples and some rotation."""
    p = np.asarray(positions, float)
    r = np.asarray(rotmats, float)
    n = len(p)
    a = np.zeros((3 * n, 6))
    a[:, :3] = r.reshape(3 * n, 3)
    a[:, 3:] = -np.tile(np.eye(3), (n, 1))
    x, _, _, sv = np.linalg.lstsq(a, -p.reshape(-1), rcond=None)
    d, t = x[:3], x[3:]
    tips = p + np.einsum('nij,j->ni', r, d)
    err = np.linalg.norm(tips - t, axis=1)
    rel = np.einsum('ji,njk->nik', r[0], r)            # R_0^T R_i
    ang = np.degrees(np.arccos(np.clip((np.trace(rel, axis1=1, axis2=2) - 1) / 2, -1, 1)))
    return {'d': d, 't': t, 'tips': tips, 'err': err, 'rmse': float(np.sqrt(np.mean(err ** 2))),
            'cond': float(sv[0] / sv[-1]) if sv[-1] > 0 else float('inf'),
            'range_deg': float(ang.max())}
