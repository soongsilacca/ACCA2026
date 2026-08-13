from __future__ import annotations

import unittest

import numpy as np
import torch

from multimodal_planner_v5.model import ModelConfig
from multimodal_planner_v7.model import RouteLockedSpeedPlannerV7
from multimodal_planner_v8.data import (
    SPATIAL_ANCHORS_M,
    temporal_residual_to_spatial_np,
)
from multimodal_planner_v8.losses import planner_loss
from multimodal_planner_v8.model import SpatialResidualPlannerV8
from multimodal_planner_v8.velocity_planner import (
    VelocityPlannerConfig,
    build_mpc_path,
    plan_curvature_speed_profile,
    plan_velocity_profile,
)


def straight_route(points: int = 121) -> np.ndarray:
    route = np.zeros((points, 2), dtype=np.float32)
    route[:, 0] = np.arange(points, dtype=np.float32)
    return route


class SpatialTargetTest(unittest.TestCase):
    def test_interpolates_only_reached_stations(self) -> None:
        progress = np.arange(1, 21, dtype=np.float32) * 2.0
        lateral = progress * 0.1
        target, valid = temporal_residual_to_spatial_np(progress, lateral)
        np.testing.assert_allclose(
            target[valid],
            SPATIAL_ANCHORS_M[valid] * 0.1,
            atol=1.0e-6,
        )
        self.assertEqual(int(valid.sum()), 13)
        self.assertTrue(np.all(target[~valid] == 0.0))

    def test_does_not_modify_source_arrays(self) -> None:
        progress = np.arange(20, dtype=np.float32)
        lateral = np.linspace(-1.0, 1.0, 20, dtype=np.float32)
        original_progress = progress.copy()
        original_lateral = lateral.copy()
        temporal_residual_to_spatial_np(progress, lateral)
        np.testing.assert_array_equal(progress, original_progress)
        np.testing.assert_array_equal(lateral, original_lateral)


class V8LossAndTransferTest(unittest.TestCase):
    def test_masked_loss_backpropagates(self) -> None:
        lateral = torch.zeros(2, 20, requires_grad=True)
        logits = torch.zeros(2, 2, requires_grad=True)
        outputs = {
            "lateral_residual_m": lateral,
            "motion_state_logits": logits,
        }
        target = torch.ones(2, 20)
        valid = torch.zeros(2, 20, dtype=torch.bool)
        valid[:, :10] = True
        total, terms = planner_loss(
            outputs,
            target,
            valid,
            torch.tensor([0, 1]),
            torch.ones(2),
        )
        total.backward()
        self.assertTrue(torch.isfinite(total))
        self.assertTrue(torch.isfinite(lateral.grad).all())
        self.assertTrue(torch.isfinite(logits.grad).all())
        self.assertAlmostEqual(float(terms["valid_anchor_fraction"]), 0.5)

    def test_v7_transfer_keeps_state_but_reinitializes_lateral(self) -> None:
        config = ModelConfig(
            hidden_dim=32,
            attention_heads=4,
            spatial_layers=1,
            route_layers=1,
            pretrained_camera=False,
        )
        v7 = RouteLockedSpeedPlannerV7(config)
        v8 = SpatialResidualPlannerV8(config)
        report = v8.load_shared_state_dict(v7.state_dict())
        self.assertTrue(report["loaded_parameters"])
        self.assertTrue(
            any(name.startswith("lateral_head.") for name in report["fresh_parameters"])
        )
        self.assertFalse(
            any(name.startswith("speed_delta_head.") for name, _ in v8.named_parameters())
        )


class MpcPathAndVelocityTest(unittest.TestCase):
    def test_builds_exact_80m_path_at_point_one_meter(self) -> None:
        residual = np.zeros(20, dtype=np.float32)
        path = build_mpc_path(
            straight_route(),
            residual,
            SPATIAL_ANCHORS_M,
        )
        self.assertEqual(path["xy_m"].shape, (801, 2))
        self.assertAlmostEqual(float(path["station_m"][-1]), 80.0, places=4)
        self.assertAlmostEqual(float(path["xy_m"][-1, 0]), 80.0, places=3)

    def test_short_route_is_rejected_instead_of_extrapolated(self) -> None:
        with self.assertRaisesRegex(ValueError, "route remains ahead"):
            build_mpc_path(
                straight_route(64),
                np.zeros(20, dtype=np.float32),
                SPATIAL_ANCHORS_M,
            )

    def test_first_residual_applies_from_path_start(self) -> None:
        route = straight_route(123)
        route[:, 0] -= 2.0
        route[:, 1] = -0.75
        path = build_mpc_path(
            route,
            np.full(20, 0.25, dtype=np.float32),
            SPATIAL_ANCHORS_M,
        )
        np.testing.assert_allclose(
            path["xy_m"][0],
            [0.0, -0.50],
            atol=1.0e-6,
        )
        self.assertAlmostEqual(float(path["xy_m"][15, 1]), -0.50, places=3)
        self.assertAlmostEqual(float(path["xy_m"][30, 1]), -0.50, places=3)
        self.assertAlmostEqual(float(path["xy_m"][-1, 1]), -0.75, places=3)

    def test_can_start_exactly_at_third_local_route_point(self) -> None:
        route = straight_route(123)
        route[:, 0] -= 2.4
        route[:, 1] = -0.75
        path = build_mpc_path(
            route,
            np.zeros(20, dtype=np.float32),
            SPATIAL_ANCHORS_M,
            route_start_index=2,
        )
        np.testing.assert_allclose(
            path["xy_m"][0],
            route[2],
            atol=1.0e-6,
        )

    def test_initial_segment_does_not_force_residual_to_zero(self) -> None:
        route = straight_route(123)
        residual = np.full(20, 1.0, dtype=np.float32)
        path = build_mpc_path(
            route,
            residual,
            SPATIAL_ANCHORS_M,
        )
        np.testing.assert_allclose(
            path["lateral_residual_m"][:31],
            1.0,
            atol=1.0e-6,
        )

    def test_small_lateral_residual_is_ignored_by_deadband(self) -> None:
        route = straight_route(123)
        residual = np.full(20, 0.08, dtype=np.float32)
        path = build_mpc_path(
            route,
            residual,
            SPATIAL_ANCHORS_M,
            lateral_deadband_m=0.1,
        )
        np.testing.assert_allclose(
            path["lateral_residual_m"],
            0.0,
            atol=1.0e-6,
        )

    def test_curvature_speed_is_capped_and_propagated_backwards(self) -> None:
        station = np.arange(0.0, 20.1, 0.1, dtype=np.float32)
        curvature = np.zeros_like(station)
        curvature[100:] = 0.1
        speed = plan_curvature_speed_profile(
            {
                "station_m": station,
                "curvature_per_m": curvature,
            },
            20.0,
            max_lateral_acceleration_mps2=3.0,
            max_deceleration_mps2=2.0,
            curvature_smoothing_m=0.0,
        )
        self.assertAlmostEqual(float(speed[100]), np.sqrt(30.0), places=5)
        self.assertLess(float(speed[0]), 20.0)
        self.assertGreater(float(speed[0]), float(speed[100]))

    def test_stop_constraint_reaches_zero(self) -> None:
        path = build_mpc_path(
            straight_route(),
            np.zeros(20, dtype=np.float32),
            SPATIAL_ANCHORS_M,
        )
        speed = plan_velocity_profile(
            path,
            current_speed_mps=10.0,
            config=VelocityPlannerConfig(competition_max_speed_mps=20.0),
            stop_distance_m=30.0,
        )
        self.assertEqual(float(speed[path["station_m"] >= 28.0].max()), 0.0)
        self.assertLessEqual(float(speed.max()), 20.0)


if __name__ == "__main__":
    unittest.main()
