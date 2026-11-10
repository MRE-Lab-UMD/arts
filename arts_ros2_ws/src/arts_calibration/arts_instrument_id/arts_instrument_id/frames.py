"""Tool frame conventions and pivot calibration.

The universal ARTS tool frame, identical for every instrument: origin at the distal-most
point of the closed jaws with the jaw at its Stage A zero; z distal along the closed-jaw
centreline; x along the jaw pivot axis; y right-handed. That convention is chosen because it
is exactly what pivot calibration physically measures, so there is nothing to interpret.

Functional points that are not the tool frame — where scissors actually cut, where forceps
actually grasp — are published as named offsets from it. See the table in
docs/instrument_identification.md.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class PivotResult:
    tip_in_marker: np.ndarray  #: (3,) tool-frame origin expressed in the marker frame
    pivot_in_world: np.ndarray  #: (3,) the divot location, a by-product but a useful check
    rms_residual: float  #: (m) how well a single fixed point explains all the poses


def pivot_calibration(rotations: np.ndarray, translations: np.ndarray) -> PivotResult:
    """Algebraic pivot calibration from marker poses swept about a stationary tip.

    ``rotations`` is (N, 3, 3) and ``translations`` (N, 3), the marker pose in the tracker
    frame. Each pose satisfies ``R_i @ p_tip + t_i = p_pivot`` for one unknown tip offset and
    one unknown pivot point, which stacks into ``[R_i  -I] [p_tip; p_pivot] = -t_i``.

    Sweep widely. Poses clustered around one orientation leave the system near-singular and
    the residual will look fine while the offset is wrong — so check ``rms_residual`` against
    the tracker's own noise floor (tens of microns for the fusionTrack), not against zero.
    """
    rotations = np.asarray(rotations, dtype=float)
    translations = np.asarray(translations, dtype=float)
    if rotations.shape[1:] != (3, 3):
        raise ValueError(f'expected (N, 3, 3) rotations, got {rotations.shape}')
    if len(rotations) != len(translations):
        raise ValueError('rotation and translation counts differ')
    if len(rotations) < 3:
        raise ValueError(f'need at least 3 poses, got {len(rotations)}')

    n = len(rotations)
    a_mat = np.zeros((3 * n, 6))
    b_vec = np.zeros(3 * n)
    for i, (rot, trans) in enumerate(zip(rotations, translations)):
        a_mat[3 * i:3 * i + 3, :3] = rot
        a_mat[3 * i:3 * i + 3, 3:] = -np.eye(3)
        b_vec[3 * i:3 * i + 3] = -trans

    solution, *_ = np.linalg.lstsq(a_mat, b_vec, rcond=None)
    residual = a_mat @ solution - b_vec

    return PivotResult(
        tip_in_marker=solution[:3],
        pivot_in_world=solution[3:],
        rms_residual=float(np.sqrt(np.mean(residual**2))),
    )


def tool_frame_from_axes(
    tip: np.ndarray,
    jaw_pivot_direction: np.ndarray,
    distal_direction: np.ndarray,
) -> np.ndarray:
    """Build the 4x4 ARTS tool frame from measured features.

    ``distal_direction`` becomes z and ``jaw_pivot_direction`` becomes x. The two will not be
    exactly perpendicular as measured, so x is re-orthogonalised against z — z wins, because
    the distal direction is the better-conditioned measurement (it comes from the shaft, over
    a long baseline, whereas the pivot axis is fitted across a few millimetres of jaw).
    """
    z = np.asarray(distal_direction, dtype=float)
    z = z / np.linalg.norm(z)
    x = np.asarray(jaw_pivot_direction, dtype=float)
    x = x - np.dot(x, z) * z
    norm = np.linalg.norm(x)
    if norm < 1e-9:
        raise ValueError('jaw pivot axis is parallel to the distal direction; check inputs')
    x = x / norm

    frame = np.eye(4)
    frame[:3, 0] = x
    frame[:3, 1] = np.cross(z, x)
    frame[:3, 2] = z
    frame[:3, 3] = np.asarray(tip, dtype=float)
    return frame
