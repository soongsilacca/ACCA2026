from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
import torch

from multimodal_planner_v5.data import (
    PhotometricAugmentation,
    RunStore,
    _apply_photometric_augmentation,
)
from multimodal_planner_v5.metrics import (
    TrajectoryMetricAccumulator,
    trajectory_diagnostics,
)
from multimodal_planner_v5.model import prepend_local_origin
from multimodal_planner_v5.outliers import TopOutlierCollector
from multimodal_planner_v5.train import _nonfinite_names, planner_loss


class PlannerLossTest(unittest.TestCase):
    @staticmethod
    def outputs(batch_size: int = 1) -> dict[str, torch.Tensor]:
        trajectory = torch.zeros(batch_size, 20, 4)
        trajectory[:, :, 0] = 0.1
        return {"trajectory": trajectory}

    def test_blackout_weight_survives_batch_size_one(self) -> None:
        target = torch.zeros(1, 20, 4)
        outputs = self.outputs()
        normal, _ = planner_loss(outputs, target, torch.tensor([1.0]), 1.0)
        blackout, _ = planner_loss(outputs, target, torch.tensor([2.0]), 1.0)
        self.assertTrue(torch.allclose(blackout, normal * 2.0))

    def test_fixed_normalizer_matches_weighted_objective(self) -> None:
        target = torch.zeros(2, 20, 4)
        outputs = self.outputs(batch_size=2)
        weights = torch.tensor([1.0, 2.0])
        total, terms = planner_loss(outputs, target, weights, float(weights.mean()))
        reconstructed = (
            terms["position"]
            + terms["start"]
            + 0.5 * terms["step"]
            + 0.25 * terms["acceleration"]
            + 0.2 * terms["yaw"]
            + 0.1 * terms["heading"]
            + 0.4 * terms["speed"]
        )
        self.assertTrue(torch.allclose(total, reconstructed))

    def test_derivative_losses_follow_gt_instead_of_forcing_straight(self) -> None:
        target = torch.zeros(1, 20, 4)
        t = torch.linspace(0.0, 1.0, 20)
        target[0, :, 0] = t
        target[0, :, 1] = 0.2 * t.square()
        exact_outputs = {"trajectory": target.clone()}
        _, exact = planner_loss(exact_outputs, target, torch.ones(1))
        self.assertEqual(float(exact["position"]), 0.0)
        self.assertEqual(float(exact["step"]), 0.0)
        self.assertEqual(float(exact["acceleration"]), 0.0)

        straight = target.clone()
        straight[..., 1] = 0.0
        _, straight_terms = planner_loss(
            {"trajectory": straight}, target, torch.ones(1)
        )
        self.assertGreater(float(straight_terms["step"]), 0.0)
        self.assertGreater(float(straight_terms["acceleration"]), 0.0)

    def test_boundary_losses_reject_constant_wrong_side_offset(self) -> None:
        target = torch.zeros(1, 20, 4)
        target[0, :, 0] = torch.linspace(0.0, 0.5, 20)
        offset = target.clone()
        offset[..., 1] = 0.1
        exact_total, _ = planner_loss(
            {"trajectory": target.clone()}, target, torch.ones(1)
        )
        offset_total, offset_terms = planner_loss(
            {"trajectory": offset}, target, torch.ones(1)
        )
        self.assertGreater(float(offset_terms["start"]), 0.0)
        self.assertGreater(float(offset_terms["step"]), 0.0)
        self.assertGreater(float(offset_terms["acceleration"]), 0.0)
        self.assertGreater(float(offset_terms["position"]), 0.0)
        self.assertGreater(float(offset_total), float(exact_total))

    def test_nonzero_first_gt_is_matched_instead_of_forced_to_origin(self) -> None:
        target = torch.zeros(1, 20, 4)
        target[0, :, 0] = torch.linspace(0.08, 1.0, 20)
        total, terms = planner_loss(
            {"trajectory": target.clone()}, target, torch.ones(1)
        )
        self.assertEqual(float(total), 0.0)
        self.assertEqual(float(terms["start"]), 0.0)
        self.assertEqual(float(terms["step"]), 0.0)
        self.assertEqual(float(terms["acceleration"]), 0.0)

    def test_backward_first_waypoint_is_penalized_at_origin_boundary(self) -> None:
        target = torch.zeros(1, 20, 4)
        target[0, :, 0] = torch.linspace(0.02, 0.4, 20)
        prediction = target.clone()
        prediction[0, 0, 0] = -0.1
        _, terms = planner_loss(
            {"trajectory": prediction}, target, torch.ones(1)
        )
        self.assertGreater(float(terms["start"]), 0.0)
        self.assertGreater(float(terms["step"]), 0.0)
        self.assertGreater(float(terms["acceleration"]), 0.0)

    def test_zigzag_increases_step_acc_and_heading_losses(self) -> None:
        target = torch.zeros(1, 20, 4)
        target[0, :, 0] = torch.linspace(0.0, 0.5, 20)
        zigzag = target.clone()
        zigzag[0, :, 1] = torch.tensor([0.03, -0.03] * 10)
        _, terms = planner_loss({"trajectory": zigzag}, target, torch.ones(1))
        self.assertGreater(float(terms["step"]), 0.0)
        self.assertGreater(float(terms["acceleration"]), 0.0)
        self.assertGreater(float(terms["heading"]), 0.0)

    def test_stationary_prediction_has_finite_heading_gradient(self) -> None:
        prediction = torch.zeros(1, 20, 4, requires_grad=True)
        target = torch.zeros_like(prediction)
        target[0, :, 0] = torch.linspace(0.01, 0.4, 20)
        loss, _ = planner_loss(
            {"trajectory": prediction}, target, torch.ones(1)
        )
        loss.backward()
        self.assertTrue(bool(torch.isfinite(prediction.grad).all()))

    def test_nonfinite_guard_names_bad_tensor(self) -> None:
        bad = _nonfinite_names(
            (
                "outputs",
                {
                    "trajectory": torch.tensor([0.0, float("nan")]),
                    "tokens": torch.ones(2),
                },
            ),
            ("loss", {"total": torch.tensor(float("inf"))}),
        )
        self.assertEqual(bad, ["outputs.trajectory", "loss.total"])


class PhotometricAugmentationTest(unittest.TestCase):
    def test_preserves_shape_dtype_and_is_deterministic(self) -> None:
        image = np.arange(24 * 32 * 3, dtype=np.uint8).reshape(24, 32, 3)
        augmentation = PhotometricAugmentation(
            brightness=1.1,
            contrast=0.85,
            saturation=1.1,
            hue_shift=0.02,
            fog_strength=0.2,
        )
        first = _apply_photometric_augmentation(image, augmentation)
        second = _apply_photometric_augmentation(image, augmentation)
        self.assertEqual(first.shape, image.shape)
        self.assertEqual(first.dtype, np.uint8)
        np.testing.assert_array_equal(first, second)
        self.assertFalse(np.array_equal(first, image))

    def test_identity_parameters_leave_image_unchanged(self) -> None:
        rng = np.random.default_rng(2026)
        image = rng.integers(0, 256, size=(24, 32, 3), dtype=np.uint8)
        result = _apply_photometric_augmentation(
            image, PhotometricAugmentation()
        )
        np.testing.assert_array_equal(result, image)

    def test_fog_reduces_contrast_without_changing_geometry(self) -> None:
        image = np.zeros((20, 30, 3), dtype=np.uint8)
        image[:, 15:] = 255
        fogged = _apply_photometric_augmentation(
            image, PhotometricAugmentation(fog_strength=0.25)
        )
        self.assertGreater(float(fogged[:, :15].mean()), 0.0)
        self.assertLess(float(fogged[:, 15:].mean()), 255.0)
        self.assertEqual(fogged.shape, image.shape)


class V5TargetContractTest(unittest.TestCase):
    @staticmethod
    def write_run(path: Path, *, v5_fields: bool) -> None:
        count = 2
        common = {
            "sample_id": np.arange(count, dtype=np.int64),
            "current_frame_idx": np.arange(count, dtype=np.int64),
            "history_frame_idx": np.zeros((count, 5), dtype=np.int64),
            "future_frame_idx": np.zeros((count, 20), dtype=np.int64),
            "gps_blackout": np.asarray([False, True]),
            "future_speed": np.full((count, 20), 4.0, dtype=np.float32),
        }
        if v5_fields:
            common.update(
                {
                    "relative_x": np.full((count, 20), 1.0, dtype=np.float32),
                    "relative_y": np.full((count, 20), 2.0, dtype=np.float32),
                    "relative_yaw": np.full((count, 20), 0.3, dtype=np.float32),
                }
            )
        else:
            common.update(
                {
                    "future_xy": np.full((count, 20, 2), 1.0, dtype=np.float32),
                    "future_yaw": np.full((count, 20), 0.3, dtype=np.float32),
                }
            )
        np.savez(path / "sample_index.npz", **common)
        (path / "frame_chunks.json").write_text(
            '{"frame_chunks": []}', encoding="utf-8"
        )

    def test_relative_fields_are_used_directly(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            run_dir = Path(temporary)
            self.write_run(run_dir, v5_fields=True)
            run = RunStore(run_dir)
            self.assertEqual(run.target_schema, "v5_relative_fields")
            self.assertEqual(run.target.shape, (2, 20, 4))
            np.testing.assert_allclose(run.target[0, 0], [1.0, 2.0, 0.3, 4.0])

    def test_legacy_fields_require_explicit_opt_in(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            run_dir = Path(temporary)
            self.write_run(run_dir, v5_fields=False)
            with self.assertRaises(KeyError):
                RunStore(run_dir)
            run = RunStore(run_dir, allow_legacy_target_fields=True)
            self.assertIn("without_numeric_correction", run.target_schema)


class V5MetricTest(unittest.TestCase):
    def test_controller_trajectory_starts_at_local_origin(self) -> None:
        trajectory = torch.randn(2, 20, 4)
        with_origin = prepend_local_origin(trajectory)
        self.assertEqual(tuple(with_origin.shape), (2, 21, 4))
        self.assertTrue(torch.equal(with_origin[:, 0, :3], torch.zeros(2, 3)))
        self.assertTrue(torch.equal(with_origin[:, 1:, :], trajectory))
        self.assertTrue(torch.equal(with_origin[:, 0, 3], trajectory[:, 0, 3]))

    def test_horizons_and_blackout_groups_without_modes(self) -> None:
        target = torch.zeros(2, 20, 4)
        target[:, :, 0] = torch.linspace(0.0, 0.5, 20)
        trajectory = target.clone()
        trajectory[0, :5, 0] += 1.0 / 50.0
        trajectory[0, 5:10, 0] += 2.0 / 50.0
        trajectory[0, 10:, 0] += 4.0 / 50.0
        diagnostics = trajectory_diagnostics({"trajectory": trajectory}, target)
        self.assertAlmostEqual(float(diagnostics["ade_1s_m"][0]), 1.0, places=5)
        self.assertAlmostEqual(float(diagnostics["fde_1s_m"][0]), 1.0, places=5)
        self.assertAlmostEqual(float(diagnostics["ade_2s_m"][0]), 1.5, places=5)
        self.assertAlmostEqual(float(diagnostics["fde_2s_m"][0]), 2.0, places=5)
        self.assertAlmostEqual(float(diagnostics["ade_4s_m"][0]), 2.75, places=5)
        self.assertAlmostEqual(float(diagnostics["fde_4s_m"][0]), 4.0, places=5)

        accumulator = TrajectoryMetricAccumulator()
        accumulator.update(diagnostics, torch.tensor([True, False]))
        result = accumulator.result()
        self.assertEqual(result["all"]["count"], 2)
        self.assertEqual(result["blackout"]["count"], 1)
        self.assertEqual(result["non_blackout"]["count"], 1)
        self.assertAlmostEqual(
            result["blackout"]["trajectory"]["ade_m"]["1s"], 1.0
        )

    def test_lateral_flip_diagnostic_uses_deadband(self) -> None:
        target = torch.zeros(1, 20, 4)
        target[0, :, 0] = torch.linspace(0.0, 0.5, 20)
        prediction = target.clone()
        prediction[0, :, 1] = torch.tensor([0.01, -0.01] * 10)
        diagnostics = trajectory_diagnostics({"trajectory": prediction}, target)
        self.assertEqual(
            float(diagnostics["gt_relative_lateral_sign_flip_count"][0]), 19.0
        )

    def test_origin_direction_mismatch_detects_backward_first_point(self) -> None:
        target = torch.zeros(1, 20, 4)
        target[0, 0, 0] = 0.02
        prediction = target.clone()
        prediction[0, 0, 0] = -0.1
        diagnostics = trajectory_diagnostics(
            {"trajectory": prediction}, target
        )
        self.assertEqual(
            float(diagnostics["origin_direction_mismatch"][0]), 1.0
        )
        self.assertAlmostEqual(
            float(diagnostics["first_waypoint_error_m"][0]), 6.0, places=5
        )


class OutlierExportTest(unittest.TestCase):
    def test_exports_gallery_and_manifest(self) -> None:
        outputs = PlannerLossTest.outputs(batch_size=2)
        outputs["trajectory"][0, :, 0] = 0.4
        outputs["trajectory"][1, :, 0] = 0.2
        target = torch.zeros(2, 20, 4)
        batch = {
            "front": torch.zeros(2, 5, 3, 36, 64),
            "left": torch.zeros(2, 5, 3, 24, 32),
            "right": torch.zeros(2, 5, 3, 24, 32),
            "target": target,
            "gps_blackout": torch.tensor([True, False]),
            "run_id": ["run-a", "run-b"],
            "sample_id": torch.tensor([10, 20]),
        }
        diagnostics = trajectory_diagnostics(outputs, target)
        collector = TopOutlierCollector(count=2)
        collector.update(outputs, batch, diagnostics)
        with tempfile.TemporaryDirectory() as temporary:
            output_root = Path(temporary)
            summary = collector.export(output_root, epoch=1)
            gallery = Path(summary["gallery"])
            manifest_path = Path(summary["manifest"])
            self.assertTrue(gallery.is_file())
            self.assertTrue(manifest_path.is_file())
            self.assertTrue((output_root / "index.html").is_file())
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(len(manifest["groups"]["overall"]), 2)
            self.assertEqual(len(manifest["groups"]["blackout"]), 1)
            for row in manifest["groups"]["overall"]:
                self.assertTrue((gallery.parent / row["image"]).is_file())


if __name__ == "__main__":
    unittest.main()
