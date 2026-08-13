from __future__ import annotations

import unittest

import numpy as np
import torch

from multimodal_planner_v5.model import ModelConfig, MultiViewTemporalTrajectoryPlannerV5
from multimodal_planner_v6.data import DRIVE, STOP
from multimodal_planner_v7.data import project_target_to_route_np
from multimodal_planner_v7.losses import planner_loss
from multimodal_planner_v7.model import (
    RouteLockedSpeedPlannerV7,
    project_points_to_route,
    speed_profile_to_trajectory,
)
from multimodal_planner_v7.train import set_training_phase


def straight_route(batch: int = 1) -> torch.Tensor:
    route = torch.zeros(batch, 64, 4)
    route[:, :, 0] = torch.arange(64).float() / 50.0
    return route


class RouteResidualGeometryTest(unittest.TestCase):
    def test_speed_integrates_to_progress(self) -> None:
        speed = torch.full((1, 20), 0.5)
        trajectory, progress = speed_profile_to_trajectory(
            speed,
            straight_route(),
        )
        self.assertAlmostEqual(float(progress[0, -1]), 40.0, places=4)
        self.assertAlmostEqual(float(trajectory[0, -1, 0]), 40.0 / 50.0, places=4)

    def test_signed_delta_moves_left_and_right(self) -> None:
        speed = torch.full((2, 20), 0.25)
        lateral = torch.stack(
            (torch.ones(20), -torch.ones(20)),
            dim=0,
        )
        trajectory, _ = speed_profile_to_trajectory(
            speed,
            straight_route(2),
            lateral,
        )
        self.assertTrue(torch.allclose(trajectory[0, :, 1], torch.full((20,), 0.02)))
        self.assertTrue(
            torch.allclose(trajectory[1, :, 1], torch.full((20,), -0.02))
        )

    def test_torch_projection_recovers_residual(self) -> None:
        route = straight_route()
        speed = torch.full((1, 20), 0.25)
        expected = torch.linspace(-1.0, 1.0, 20).unsqueeze(0)
        trajectory, progress = speed_profile_to_trajectory(
            speed,
            route,
            expected,
        )
        projected_progress, lateral = project_points_to_route(
            route,
            trajectory[..., :2] * 50.0,
        )
        self.assertTrue(torch.allclose(progress, projected_progress, atol=1.0e-4))
        self.assertTrue(torch.allclose(expected, lateral, atol=1.0e-4))

    def test_numpy_projection_does_not_modify_target(self) -> None:
        route = np.zeros((64, 4), dtype=np.float32)
        route[:, 0] = np.arange(64)
        target = np.zeros((20, 4), dtype=np.float32)
        target[:, 0] = np.arange(1, 21)
        target[:, 1] = 1.5
        original = target.copy()
        progress, lateral = project_target_to_route_np(route, target)
        np.testing.assert_array_equal(target, original)
        np.testing.assert_allclose(progress, np.arange(1, 21), atol=1.0e-5)
        np.testing.assert_allclose(lateral, 1.5, atol=1.0e-5)


class V7TransferAndLossTest(unittest.TestCase):
    def test_v5_encoder_transfer_ignores_old_trajectory_head(self) -> None:
        config = ModelConfig(
            hidden_dim=32,
            attention_heads=4,
            spatial_layers=1,
            route_layers=1,
            pretrained_camera=False,
            freeze_camera_backbone=True,
        )
        v5 = MultiViewTemporalTrajectoryPlannerV5(config)
        v7 = RouteLockedSpeedPlannerV7(config)
        report = v7.load_encoder_state_dict(v5.state_dict())
        self.assertTrue(report["loaded_encoder_parameters"])
        self.assertTrue(
            any(name.startswith("trajectory_head.") for name in report["ignored_source_parameters"])
        )

    def test_loss_is_finite_and_backpropagates_to_speed_and_delta(self) -> None:
        future_speed = torch.full((2, 20), 0.25, requires_grad=True)
        lateral = torch.zeros(2, 20, requires_grad=True)
        trajectory, progress = speed_profile_to_trajectory(
            future_speed,
            straight_route(2),
            lateral,
        )
        logits = torch.zeros(2, 2, requires_grad=True)
        outputs = {
            "trajectory": trajectory,
            "future_speed": future_speed,
            "base_speed": future_speed.detach(),
            "speed_delta": torch.zeros_like(future_speed),
            "lateral_residual_m": lateral,
            "forward_progress_m": progress,
            "motion_state_logits": logits,
        }
        target = trajectory.detach().clone()
        total, _ = planner_loss(
            outputs,
            target,
            progress.detach(),
            lateral.detach(),
            torch.tensor([STOP, DRIVE]),
            torch.ones(2),
        )
        total.backward()
        self.assertTrue(torch.isfinite(total))
        self.assertTrue(torch.isfinite(future_speed.grad).all())
        self.assertTrue(torch.isfinite(lateral.grad).all())
        self.assertTrue(torch.isfinite(logits.grad).all())

    def test_head_warmup_only_trains_new_heads(self) -> None:
        config = ModelConfig(
            hidden_dim=32,
            attention_heads=4,
            pretrained_camera=False,
        )
        model = RouteLockedSpeedPlannerV7(config)
        phase = set_training_phase(model, 0, 1, 2)
        self.assertEqual(phase, "new_heads_only")
        self.assertTrue(all(p.requires_grad for p in model.lateral_head.parameters()))
        self.assertTrue(
            all(not p.requires_grad for p in model.route_encoder.parameters())
        )

    def test_mgeo_base_speed_is_fixed_and_has_no_parameters(self) -> None:
        config = ModelConfig(
            hidden_dim=32,
            attention_heads=4,
            pretrained_camera=False,
        )
        model = RouteLockedSpeedPlannerV7(
            config,
            normal_base_speed_mps=10.0,
            speed_zone_base_speed_mps=20.0,
        )
        mgeo = torch.zeros(2, 64, 8, requires_grad=True)
        with torch.no_grad():
            mgeo[1, :16, 7] = 1.0
        base = model.fixed_mgeo_base_speed(mgeo)
        self.assertTrue(torch.allclose(base[0], torch.full((20,), 10.0)))
        self.assertTrue(torch.allclose(base[1], torch.full((20,), 20.0)))
        self.assertFalse(
            any(name.startswith("base_speed") for name, _ in model.named_parameters())
        )


if __name__ == "__main__":
    unittest.main()
