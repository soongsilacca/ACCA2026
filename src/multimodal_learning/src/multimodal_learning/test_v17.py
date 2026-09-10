from __future__ import annotations

import unittest

import numpy as np
import torch

from multimodal_planner_v9.model import ModelConfig
from .data import (
    SPATIAL_ANCHORS_M,
    route_points_and_normals_np,
    temporal_speed_to_spatial_np,
)
from .model import GoalSpatialCandidatePlannerV17


class V17SpatialContractTest(unittest.TestCase):
    def test_anchor_contract(self) -> None:
        np.testing.assert_allclose(SPATIAL_ANCHORS_M, [3, 6, 10, 15, 22, 30])

    def test_straight_route_points_and_normals(self) -> None:
        route = np.zeros((64, 4), dtype=np.float32)
        route[:, 0] = np.arange(64, dtype=np.float32) / 50.0
        points, normals = route_points_and_normals_np(route)
        np.testing.assert_allclose(points[:, 0], SPATIAL_ANCHORS_M, atol=1e-4)
        np.testing.assert_allclose(points[:, 1], 0.0, atol=1e-5)
        np.testing.assert_allclose(normals, np.tile([0.0, 1.0], (6, 1)), atol=1e-4)

    def test_speed_is_spatial_and_masks_unreached_stations(self) -> None:
        target, valid = temporal_speed_to_spatial_np(
            np.arange(1, 21, dtype=np.float32),
            np.linspace(2, 10, 20, dtype=np.float32),
        )
        np.testing.assert_array_equal(valid, [True, True, True, True, False, False])
        self.assertTrue(np.all(target[valid] > 0.0))
        np.testing.assert_allclose(target[~valid], 0.0)

    def test_model_starts_at_fixed_spatial_stations(self) -> None:
        model = GoalSpatialCandidatePlannerV17(
            ModelConfig(pretrained_camera=False, freeze_camera_backbone=True)
        ).eval()
        x = torch.zeros(1, 1, 3, 64, 64)
        with torch.inference_mode():
            outputs = model(x, x, x, x, torch.tensor([[1.0, 0.0]]))
        np.testing.assert_allclose(
            outputs["drive_path_xy_m"][0, :, 0].numpy(),
            SPATIAL_ANCHORS_M, atol=1e-5,
        )
        np.testing.assert_allclose(outputs["drive_path_xy_m"][0, :, 1].numpy(), 0.0)
        self.assertEqual(tuple(outputs["atomic_output"].shape), (1, 33))


if __name__ == "__main__":
    unittest.main()
