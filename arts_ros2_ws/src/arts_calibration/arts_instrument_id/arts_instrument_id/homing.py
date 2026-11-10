"""Stage A: statistics over repeated hard-stop homing cycles.

The drive-until-current-peaks search itself is hardware and lives in the capture node; this
module only summarises its output. That split is deliberate — the statistics are what get
published, and they must be reproducible from committed data without a robot attached.

Actuator zero is the reference for every parameter identified later, so its repeatability is
the noise floor on all of them. Two numbers matter and they are different: back-to-back
repeatability, and repeatability across re-seating the instrument on the adapter. The second
is the one that matters operationally and it will be worse.

See docs/instrument_identification.md, Stage A.
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class HomingCycle:
    """One bisection: both detected hard stops for one disc, in radians."""

    disc: str
    stop_positive: float
    stop_negative: float
    seating_id: int = 0  #: increments when the instrument is removed and re-seated
    free_current: float = float('nan')  #: (A) measured before the search, for the threshold
    peak_current: float = float('nan')  #: (A) at detection

    @property
    def zero(self) -> float:
        return 0.5 * (self.stop_positive + self.stop_negative)

    @property
    def travel(self) -> float:
        return abs(self.stop_positive - self.stop_negative)


@dataclass
class HomingStats:
    disc: str
    zero_mean: float
    zero_std: float
    zero_ptp: float
    travel_mean: float
    reseat_std: float  #: std of the per-seating mean zeros; nan if only one seating
    n_cycles: int
    n_seatings: int
    asymmetry: float = field(default=float('nan'))

    def summary(self) -> str:
        reseat = (
            'single seating'
            if np.isnan(self.reseat_std)
            else f're-seat std {np.rad2deg(self.reseat_std):.4f} deg'
        )
        return (
            f'{self.disc}: zero std {np.rad2deg(self.zero_std):.4f} deg over '
            f'{self.n_cycles} cycles, {reseat}'
        )


def summarize(cycles: list[HomingCycle], reference_zero: float | None = None) -> HomingStats:
    """Repeatability statistics for one disc.

    ``asymmetry`` is the offset of the detected midpoint from ``reference_zero`` (typically
    the mechanical centre from CAD or calipers) when one is supplied. It is reported rather
    than corrected: friction is not symmetric, the two detected stops are therefore not
    symmetric about the true centre, and hiding that inside the midpoint would launder a
    systematic error into an apparently clean zero.
    """
    if not cycles:
        raise ValueError('no homing cycles supplied')
    discs = {c.disc for c in cycles}
    if len(discs) != 1:
        raise ValueError(f'summarize() handles one disc at a time, got {sorted(discs)}')

    zeros = np.array([c.zero for c in cycles])
    seatings = np.array([c.seating_id for c in cycles])

    unique = np.unique(seatings)
    reseat_std = (
        float(np.std([zeros[seatings == s].mean() for s in unique], ddof=1))
        if len(unique) > 1
        else float('nan')
    )

    return HomingStats(
        disc=cycles[0].disc,
        zero_mean=float(zeros.mean()),
        zero_std=float(zeros.std(ddof=1)) if len(zeros) > 1 else 0.0,
        zero_ptp=float(zeros.ptp()),
        travel_mean=float(np.mean([c.travel for c in cycles])),
        reseat_std=reseat_std,
        n_cycles=len(cycles),
        n_seatings=len(unique),
        asymmetry=(
            float(zeros.mean() - reference_zero) if reference_zero is not None else float('nan')
        ),
    )
