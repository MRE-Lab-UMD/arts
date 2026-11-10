"""Circle-point analysis: recover a joint axis and per-sample joint angles from the
path swept by a single tracked point.

CROSS-CHECK ONLY, AND ONLY AFTER THE TEARDOWN. This is not the primary identification
method and must not be used as one. A da Vinci disc does not drive a joint — the wrist is a
coupled cable transmission, which is why a coupling matrix exists at all — so actuating one
disc moves several joints and the tip traces a compound path rather than a circle. Worse, you
cannot design a single-joint excitation without already knowing the coupling you are trying
to measure.

What makes this usable at all is Step 1 of the protocol: tearing down an expired instrument
establishes the transmission structure by direct observation, and structural knowledge (shaft
roll has its own disc; discs 3 and 4 are a differential pair) lets you command combinations
that genuinely move one joint. Fit those, and compare against the regression in coupling.py.

Given a genuinely single-joint sweep, the maths is as advertised: the swept path is a circle
whose plane normal is the joint axis, whose centre lies on the axis, and whose radius is the
perpendicular distance from axis to the tracked point.

See docs/instrument_identification.md, Stage B.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Axis:
    """A revolute axis recovered from a swept point path, in the frame of the input points."""

    point: np.ndarray  #: (3,) circle centre, which lies on the axis
    direction: np.ndarray  #: (3,) unit vector along the axis
    radius: float  #: perpendicular distance from the axis to the swept point
    rms_residual: float  #: (m) RMS distance of the samples from the fitted circle
    planarity: float  #: (m) RMS out-of-plane deviation; see note below

    def __repr__(self) -> str:
        return (
            f'Axis(radius={self.radius * 1e3:.3f} mm, '
            f'rms={self.rms_residual * 1e6:.1f} um, '
            f'planarity={self.planarity * 1e6:.1f} um)'
        )


def fit_plane(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Total-least-squares plane through ``points`` (N, 3).

    Returns ``(centroid, unit_normal)``. The normal is the least-significant right singular
    vector, i.e. the direction of least variance.
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f'expected (N, 3) points, got {points.shape}')
    if len(points) < 3:
        raise ValueError(f'need at least 3 points to fit a plane, got {len(points)}')

    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
    return centroid, vt[-1] / np.linalg.norm(vt[-1])


def fit_circle_3d(points: np.ndarray) -> Axis:
    """Fit a circle to ``points`` (N, 3) and return the implied revolute axis.

    The in-plane fit is the standard linearisation: for centre ``(a, b)`` and radius ``r``,
    ``x^2 + y^2 = 2ax + 2by + (r^2 - a^2 - b^2)`` is linear in ``(a, b, c)``, so one least
    squares solve suffices. Good conditioning needs a decent swept arc — a few degrees of
    travel will fit a circle of almost any radius, so check ``radius`` against the caliper
    value rather than trusting ``rms_residual`` alone.

    ``planarity`` is reported separately from ``rms_residual`` on purpose. During a nominally
    single-joint sweep, out-of-plane deviation is a direct readout of cross-coupling: another
    joint moved when it should not have. That is a measurement, not a failed fit.
    """
    points = np.asarray(points, dtype=float)
    if len(points) < 4:
        raise ValueError(f'need at least 4 points to fit a circle, got {len(points)}')

    centroid, normal = fit_plane(points)

    # Orthonormal in-plane basis. Seed from whichever world axis is least parallel to the
    # normal so the cross product never degenerates.
    seed = np.eye(3)[int(np.argmin(np.abs(normal)))]
    u = np.cross(normal, seed)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)

    local = points - centroid
    x, y = local @ u, local @ v
    out_of_plane = local @ normal

    a_mat = np.column_stack([2.0 * x, 2.0 * y, np.ones_like(x)])
    sol, *_ = np.linalg.lstsq(a_mat, x**2 + y**2, rcond=None)
    cx, cy, c = sol
    radius = float(np.sqrt(max(c + cx**2 + cy**2, 0.0)))

    in_plane_residual = np.hypot(x - cx, y - cy) - radius
    rms = float(np.sqrt(np.mean(in_plane_residual**2 + out_of_plane**2)))

    return Axis(
        point=centroid + cx * u + cy * v,
        direction=normal,
        radius=radius,
        rms_residual=rms,
        planarity=float(np.sqrt(np.mean(out_of_plane**2))),
    )


def angles_about_axis(
    points: np.ndarray,
    axis: Axis,
    reference: int = 0,
) -> np.ndarray:
    """Signed angle (rad) of each point about ``axis``, relative to ``points[reference]``.

    Unwrapped, so a sweep past +/-pi stays monotonic. The sign follows the right-hand rule
    about ``axis.direction``; since that direction comes from an SVD its sign is arbitrary,
    so fix the convention per joint in the instrument config rather than trusting the fit.
    """
    points = np.asarray(points, dtype=float)
    normal = axis.direction

    seed = np.eye(3)[int(np.argmin(np.abs(normal)))]
    u = np.cross(normal, seed)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)

    radial = points - axis.point
    radial = radial - np.outer(radial @ normal, normal)
    raw = np.unwrap(np.arctan2(radial @ v, radial @ u))
    return raw - raw[reference]
