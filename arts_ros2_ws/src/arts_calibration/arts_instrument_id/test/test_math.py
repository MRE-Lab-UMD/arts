"""Synthetic-data checks for the offline math.

These run without hardware, which is the point: the analysis has to be verifiable from
committed data alone. Tolerances are set near the fusionTrack's own noise floor so a
regression that matters is caught and float noise is not.
"""

import numpy as np
import pytest

from arts_instrument_id.axis_fit import angles_about_axis, fit_circle_3d, fit_plane
from arts_instrument_id.coupling import backlash, fit_coupling
from arts_instrument_id.frames import pivot_calibration, tool_frame_from_axes
from arts_instrument_id.geometry import observability
from arts_instrument_id.homing import HomingCycle, summarize


def _orthonormal_basis(normal):
    normal = normal / np.linalg.norm(normal)
    seed = np.eye(3)[int(np.argmin(np.abs(normal)))]
    u = np.cross(normal, seed)
    u /= np.linalg.norm(u)
    return u, np.cross(normal, u), normal


class TestAxisFit:
    def test_plane_normal_recovered(self):
        rng = np.random.default_rng(0)
        normal = np.array([0.3, -0.5, 0.81])
        normal /= np.linalg.norm(normal)
        u, v, _ = _orthonormal_basis(normal)
        pts = np.array([1.0, 2.0, 3.0]) + rng.normal(size=(60, 2)) @ np.vstack([u, v])

        _, fitted = fit_plane(pts)
        assert abs(abs(np.dot(fitted, normal)) - 1.0) < 1e-9

    def test_circle_recovers_axis_and_radius(self):
        centre = np.array([0.01, -0.02, 0.15])
        normal = np.array([0.0, 1.0, 0.0])
        radius = 0.0091  # a plausible da Vinci pitch-to-yaw offset, in metres
        u, v, _ = _orthonormal_basis(normal)

        theta = np.linspace(-1.0, 1.0, 80)
        pts = centre + radius * (np.outer(np.cos(theta), u) + np.outer(np.sin(theta), v))

        axis = fit_circle_3d(pts)
        assert axis.radius == pytest.approx(radius, abs=1e-9)
        assert np.linalg.norm(axis.point - centre) < 1e-9
        assert abs(abs(np.dot(axis.direction, normal)) - 1.0) < 1e-9
        assert axis.planarity < 1e-12

    def test_circle_survives_tracker_noise(self):
        """20 um isotropic noise, roughly the fusionTrack's single-fiducial scatter."""
        rng = np.random.default_rng(7)
        centre = np.zeros(3)
        radius = 0.0091
        u, v, _ = _orthonormal_basis(np.array([0.0, 0.0, 1.0]))
        theta = np.linspace(-1.2, 1.2, 200)
        pts = centre + radius * (np.outer(np.cos(theta), u) + np.outer(np.sin(theta), v))
        pts = pts + rng.normal(scale=20e-6, size=pts.shape)

        axis = fit_circle_3d(pts)
        assert axis.radius == pytest.approx(radius, abs=20e-6)

    def test_angles_match_commanded_sweep(self):
        normal = np.array([0.0, 0.0, 1.0])
        u, v, _ = _orthonormal_basis(normal)
        theta = np.linspace(-0.8, 0.8, 50)
        pts = 0.01 * (np.outer(np.cos(theta), u) + np.outer(np.sin(theta), v))

        axis = fit_circle_3d(pts)
        recovered = angles_about_axis(pts, axis)
        # Sign follows the SVD's arbitrary normal direction, hence the abs().
        assert np.allclose(np.abs(recovered), np.abs(theta - theta[0]), atol=1e-9)

    def test_rejects_degenerate_input(self):
        with pytest.raises(ValueError):
            fit_circle_3d(np.zeros((3, 3)))
        with pytest.raises(ValueError):
            fit_plane(np.zeros((2, 3)))


class TestCoupling:
    def test_recovers_known_matrix(self):
        rng = np.random.default_rng(1)
        truth = np.array([
            [-1.5632, 0.0, 0.0, 0.0],
            [0.0, 1.0186, 0.0, 0.0],
            [0.0, -0.8306, 0.6089, 0.6089],
            [0.0, -0.8306, -0.6089, 0.6089],
        ])
        q_act = rng.uniform(-1.0, 1.0, size=(400, 4))
        q_joint = q_act @ truth.T

        fit = fit_coupling(q_act, q_joint)
        assert np.allclose(fit.matrix, truth, atol=1e-10)
        assert fit.residual_rms.max() < 1e-12

    def test_flags_poor_excitation(self):
        """Discs moved together produce a confident-looking matrix that is mostly noise."""
        base = np.linspace(-1, 1, 200)
        q_act = np.column_stack([base, base, base, base]) + 1e-9
        fit = fit_coupling(q_act, q_act @ np.eye(4))
        assert fit.condition_number > 1e6

    def test_rejects_mismatched_samples(self):
        with pytest.raises(ValueError):
            fit_coupling(np.zeros((10, 4)), np.zeros((9, 4)))

    def test_backlash_band_matches_injected_hysteresis(self):
        sweep = np.linspace(-1.0, 1.0, 300)
        gap = np.deg2rad(0.8)
        up = np.column_stack([sweep + gap / 2])
        down = np.column_stack([sweep - gap / 2])

        result = backlash(
            np.concatenate([sweep, sweep]),
            np.vstack([up, down]),
            np.concatenate([np.ones(300, bool), np.zeros(300, bool)]),
        )
        assert result['band'][0] == pytest.approx(gap, rel=1e-6)

    def test_backlash_needs_both_directions(self):
        with pytest.raises(ValueError):
            backlash(np.linspace(0, 1, 10), np.zeros((10, 1)), np.ones(10, bool))


class TestFrames:
    def test_pivot_recovers_tip_offset(self):
        rng = np.random.default_rng(3)
        tip_in_marker = np.array([0.0, 0.0, -0.012])
        pivot = np.array([0.4, 0.1, -0.3])

        rots, trans = [], []
        for _ in range(60):
            axis = rng.normal(size=3)
            axis /= np.linalg.norm(axis)
            angle = rng.uniform(-0.7, 0.7)
            k = np.array([[0, -axis[2], axis[1]],
                          [axis[2], 0, -axis[0]],
                          [-axis[1], axis[0], 0]])
            rot = np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k
            rots.append(rot)
            trans.append(pivot - rot @ tip_in_marker)

        result = pivot_calibration(np.array(rots), np.array(trans))
        assert np.allclose(result.tip_in_marker, tip_in_marker, atol=1e-9)
        assert np.allclose(result.pivot_in_world, pivot, atol=1e-9)
        assert result.rms_residual < 1e-9

    def test_tool_frame_is_orthonormal_and_reorthogonalises(self):
        # Deliberately non-perpendicular inputs, as measurements always are.
        frame = tool_frame_from_axes(
            tip=[0.0, 0.0, 0.4],
            jaw_pivot_direction=[1.0, 0.0, 0.1],
            distal_direction=[0.0, 0.0, 1.0],
        )
        rot = frame[:3, :3]
        assert np.allclose(rot @ rot.T, np.eye(3), atol=1e-12)
        assert np.linalg.det(rot) == pytest.approx(1.0)
        assert np.allclose(rot[:, 2], [0, 0, 1])  # z is preserved; x gives way

    def test_rejects_parallel_axes(self):
        with pytest.raises(ValueError):
            tool_frame_from_axes([0, 0, 0], [0, 0, 1], [0, 0, 1])


class TestObservability:
    def test_flags_unidentifiable_parameter(self):
        rng = np.random.default_rng(5)
        jac = rng.normal(size=(200, 4))
        jac[:, 3] *= 1e-9  # a parameter the poses do not constrain

        result = observability(jac)
        assert result.identifiable[:3].all()
        assert not result.identifiable[3]

    def test_rejects_underdetermined(self):
        with pytest.raises(ValueError):
            observability(np.zeros((3, 8)))


class TestHoming:
    def test_reseat_std_separates_from_cycle_std(self):
        cycles = []
        for seating, bias in enumerate([0.0, 0.01, -0.01]):
            for i in range(20):
                jitter = 1e-4 * ((i % 5) - 2)
                cycles.append(
                    HomingCycle(
                        disc='disc_1',
                        stop_positive=1.0 + bias + jitter,
                        stop_negative=-1.0 + bias + jitter,
                        seating_id=seating,
                    )
                )

        stats = summarize(cycles, reference_zero=0.0)
        assert stats.n_seatings == 3
        assert stats.n_cycles == 60
        # Re-seating dominates: that is the operationally relevant number.
        assert stats.reseat_std > stats.zero_std / 2
        assert stats.travel_mean == pytest.approx(2.0, abs=1e-9)

    def test_single_seating_reports_nan_not_zero(self):
        cycles = [HomingCycle('disc_1', 1.0, -1.0) for _ in range(5)]
        assert np.isnan(summarize(cycles).reseat_std)

    def test_rejects_mixed_discs(self):
        with pytest.raises(ValueError):
            summarize([HomingCycle('disc_1', 1, -1), HomingCycle('disc_2', 1, -1)])
