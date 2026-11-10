"""Stage B: actuator-to-joint coupling matrix and backlash.

The da Vinci wrist is cable-driven, so disc rotations do not map one-to-one onto joint
angles. Under the constant-matrix model, ``q_joint = C @ q_actuator``, which makes the fit a
linear least squares. Backlash is deliberately *not* folded into C: it is extracted from
bidirectional sweeps and reported on its own, because it bounds achievable tip accuracy and
nobody else in this space publishes it.

See docs/instrument_identification.md, Stage B.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class CouplingFit:
    matrix: np.ndarray  #: (J, A) actuator-to-joint map
    residual_rms: np.ndarray  #: (J,) per-joint residual, same units as q_joint
    condition_number: float  #: of the actuator excitation matrix; see note in fit_coupling
    n_samples: int

    def summary(self) -> str:
        worst = np.rad2deg(self.residual_rms.max())
        return (
            f'{self.n_samples} samples, worst per-joint residual {worst:.4f} deg, '
            f'excitation condition number {self.condition_number:.1f}'
        )


def fit_coupling(q_actuator: np.ndarray, q_joint: np.ndarray) -> CouplingFit:
    """Least-squares fit of ``q_joint = C @ q_actuator``.

    ``q_actuator`` is (N, A) disc angles, ``q_joint`` is (N, J) observed joint angles, both
    in radians and both referenced to the Stage A zero.

    **Read the condition number before believing the matrix.** A poorly excited dataset — one
    where the discs were never moved independently enough — produces a confident-looking C
    that is mostly noise. Single-disc sweeps exist to keep this number low; if it is large,
    the fix is more diverse excitation, not a different solver.
    """
    q_actuator = np.atleast_2d(np.asarray(q_actuator, dtype=float))
    q_joint = np.atleast_2d(np.asarray(q_joint, dtype=float))
    if len(q_actuator) != len(q_joint):
        raise ValueError(
            f'sample count mismatch: {len(q_actuator)} actuator vs {len(q_joint)} joint'
        )
    if len(q_actuator) < q_actuator.shape[1]:
        raise ValueError(
            f'need at least {q_actuator.shape[1]} samples to fit, got {len(q_actuator)}'
        )

    # lstsq solves q_actuator @ X = q_joint, so X is C transposed.
    solution, *_ = np.linalg.lstsq(q_actuator, q_joint, rcond=None)
    matrix = solution.T

    residual = q_joint - q_actuator @ solution
    return CouplingFit(
        matrix=matrix,
        residual_rms=np.sqrt(np.mean(residual**2, axis=0)),
        condition_number=float(np.linalg.cond(q_actuator)),
        n_samples=len(q_actuator),
    )


def backlash(
    q_actuator: np.ndarray,
    q_joint: np.ndarray,
    increasing: np.ndarray,
    n_bins: int = 20,
) -> dict[str, np.ndarray]:
    """Hysteresis band per joint, from a bidirectional sweep.

    ``increasing`` is a boolean mask marking samples taken while the actuator was moving in
    the positive direction. Both directions are binned over the shared actuator range and the
    band is the mean gap between the two branches.

    Returns ``band`` (J,) in radians, plus the binned branches so the loop can be plotted —
    plot it at least once per instrument, because a band that varies strongly across the range
    means cable tension is uneven and is worth chasing before trusting any of the numbers.
    """
    q_actuator = np.asarray(q_actuator, dtype=float)
    q_joint = np.atleast_2d(np.asarray(q_joint, dtype=float))
    increasing = np.asarray(increasing, dtype=bool)

    if q_actuator.ndim != 1:
        raise ValueError('backlash expects a single swept actuator column, got '
                         f'shape {q_actuator.shape}')
    if not increasing.any() or increasing.all():
        raise ValueError('need samples in both directions to measure backlash')

    lo = max(q_actuator[increasing].min(), q_actuator[~increasing].min())
    hi = min(q_actuator[increasing].max(), q_actuator[~increasing].max())
    if not hi > lo:
        raise ValueError('the two sweep directions do not overlap in actuator range')

    edges = np.linspace(lo, hi, n_bins + 1)
    centres = 0.5 * (edges[:-1] + edges[1:])

    def branch(mask: np.ndarray) -> np.ndarray:
        out = np.full((n_bins, q_joint.shape[1]), np.nan)
        idx = np.digitize(q_actuator[mask], edges) - 1
        for b in range(n_bins):
            sel = idx == b
            if sel.any():
                out[b] = q_joint[mask][sel].mean(axis=0)
        return out

    up, down = branch(increasing), branch(~increasing)
    gap = np.abs(up - down)

    return {
        'actuator_bin_centres': centres,
        'branch_increasing': up,
        'branch_decreasing': down,
        'band': np.nanmean(gap, axis=0),
        'band_max': np.nanmax(gap, axis=0),
    }
