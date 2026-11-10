"""Identification of da Vinci X instrument kinematics from the physical instrument.

Protocol and rationale: ``docs/instrument_identification.md``.
Licensing verdict on why nothing is reused: catalog ``parts/4_tools/NOTES.md``.
"""

from arts_instrument_id import axis_fit, coupling, frames, geometry, homing, report

__all__ = ['axis_fit', 'coupling', 'frames', 'geometry', 'homing', 'report']
