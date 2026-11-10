"""Stage C: link geometry refinement and observability analysis.

The refined fit needs a kinematic model to differentiate, which arrives with pinocchio in
Phase 2. What is implementable now — and what actually decides whether the fit can be trusted
— is the observability analysis, so that is here and the fit is a documented stub.

See docs/instrument_identification.md, Stage C.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Observability:
    singular_values: np.ndarray
    o1_index: float  #: Borm and Menq's index; larger is better excited
    condition_number: float
    identifiable: np.ndarray  #: (P,) bool mask over parameters
    threshold: float

    def report(self, names: list[str]) -> str:
        lines = [f'O1 = {self.o1_index:.4e}, cond = {self.condition_number:.1f}']
        for name, sv, ok in zip(names, self.singular_values, self.identifiable):
            lines.append(f'  {"OK " if ok else "DROP"}  {name:<28} sigma = {sv:.4e}')
        return '\n'.join(lines)


def observability(jacobian: np.ndarray, ratio: float = 1e-3) -> Observability:
    """Which parameters are actually identifiable from the poses that were collected.

    ``jacobian`` is (M, P): the derivative of every stacked measurement with respect to every
    candidate parameter, evaluated over the whole pose set.

    Parameters whose singular value falls below ``ratio`` times the largest are in the
    near-null space — the data does not constrain them. Fitting them anyway produces
    precise-looking numbers that are fitting noise, which is the standard way a calibration
    paper ends up reporting a link length to four decimals that is wrong in the second. Fix
    those at the caliper value and say so in the report.

    ``ratio`` at 1e-3 is a starting point, not a law. Choose it by looking at where the
    singular value spectrum actually falls off for your pose set and record the choice.
    """
    jacobian = np.atleast_2d(np.asarray(jacobian, dtype=float))
    m, p = jacobian.shape
    if m < p:
        raise ValueError(
            f'jacobian is under-determined: {m} measurements for {p} parameters; '
            'collect more poses'
        )

    sv = np.linalg.svd(jacobian, compute_uv=False)
    threshold = sv[0] * ratio

    return Observability(
        singular_values=sv,
        o1_index=float(np.prod(sv) ** (1.0 / p) / np.sqrt(m)),
        condition_number=float(sv[0] / sv[-1]) if sv[-1] > 0 else float('inf'),
        identifiable=sv >= threshold,
        threshold=float(threshold),
    )


def refine_link_geometry(*_args, **_kwargs):
    """Levenberg-Marquardt fit of link parameters to tracked tip poses. NOT IMPLEMENTED.

    Blocked on the pinocchio instrument model (Phase 2). When it lands, three things decide
    whether the output is worth anything, and all three are easy to omit by accident:

    1. Solve for the tip-to-marker transform as free parameters. Fixing it at a nominal value
       pushes that error into the link lengths instead, where it is invisible.
    2. Run ``observability()`` first and freeze the unidentifiable parameters at their caliper
       values rather than letting the optimiser fit noise into them.
    3. Hold out 30% of the poses and report error on those. Training residual alone is not a
       result.

    Initialise from the circle fits in ``axis_fit`` and the caliper measurements; that is what
    keeps the solve out of local minima.
    """
    raise NotImplementedError(
        'refine_link_geometry() is Phase 2 work, blocked on the pinocchio instrument model. '
        'Until then use caliper geometry plus axis_fit.fit_circle_3d(). '
        'See docs/instrument_identification.md, Stage C.'
    )
