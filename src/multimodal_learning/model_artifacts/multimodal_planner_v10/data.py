from __future__ import annotations

from typing import Any

import numpy as np
import torch

from multimodal_planner_v9.data import (
    ACTION_DRIVE,
    ACTION_STOP,
    FIXED_BASE_SPEED_MPS,
    TARGET_POLICY_MORAI_ROUTE_RESIDUAL,
    PlannerDataset,
)


# Raw trajectory labels are sampled every 0.2 s. These are direct label
# selections, not interpolated values.
TEMPORAL_ANCHORS_S = np.asarray(
    [0.6, 1.0, 1.6, 2.0, 3.0, 4.0], dtype=np.float32
)
TEMPORAL_TARGET_INDICES = np.asarray([2, 4, 7, 9, 14, 19], dtype=np.int64)


class TemporalPlannerDataset(PlannerDataset):
    """V9 reviewed action labels with six time-indexed residual targets."""

    def __getitem__(self, item: int) -> dict[str, Any]:
        sample = super().__getitem__(item)
        run_index, sample_index = self.lookup[item]
        run = self.runs[run_index]
        action = int(self.action_labels[item])

        lateral = self.lateral_residual_targets[
            item, TEMPORAL_TARGET_INDICES
        ].astype(np.float32, copy=True)
        future_speed = np.asarray(
            run.target[sample_index, TEMPORAL_TARGET_INDICES, 3],
            dtype=np.float32,
        )
        speed = np.clip(
            future_speed - FIXED_BASE_SPEED_MPS,
            -FIXED_BASE_SPEED_MPS,
            0.0,
        ).astype(np.float32)
        lateral_valid = np.ones(6, dtype=np.bool_)
        speed_valid = np.ones(6, dtype=np.bool_)

        if action == ACTION_STOP:
            lateral_valid[:] = False
            speed_valid[:] = False
        elif (
            self.target_policy == TARGET_POLICY_MORAI_ROUTE_RESIDUAL
            and action == ACTION_DRIVE
        ):
            lateral[:] = 0.0
            speed[:] = 0.0

        # Keep V9 key names so its proven loss/metric loop remains reusable;
        # values are temporal [6], not spatial [20].
        sample["target_spatial_lateral_m"] = torch.from_numpy(lateral)
        sample["target_spatial_valid"] = torch.from_numpy(lateral_valid)
        sample["target_spatial_speed_delta_mps"] = torch.from_numpy(speed)
        sample["target_spatial_speed_valid"] = torch.from_numpy(speed_valid)
        sample["target_temporal_lateral_m"] = torch.from_numpy(lateral.copy())
        sample["target_temporal_speed_delta_mps"] = torch.from_numpy(speed.copy())
        sample["target_temporal_valid"] = torch.from_numpy(lateral_valid.copy())
        sample["temporal_anchors_s"] = torch.from_numpy(TEMPORAL_ANCHORS_S.copy())
        sample["base_speed_mps"] = torch.full(
            (6,), FIXED_BASE_SPEED_MPS, dtype=torch.float32
        )
        return sample

    def summary(self) -> dict[str, Any]:
        result = super().summary()
        result.pop("spatial_anchors_m", None)
        result.pop("valid_labels_per_anchor", None)
        result.pop("mean_valid_anchors_per_sample", None)
        result.update(
            {
                "architecture_target": (
                    "V10_speed_masked_classifier_plus_6x4s_delta_d_delta_v"
                ),
                "residual_axis": "time",
                "temporal_anchors_s": TEMPORAL_ANCHORS_S.tolist(),
                "temporal_target_indices": TEMPORAL_TARGET_INDICES.tolist(),
                "classifier_ego_policy": (
                    "all 16 ego/localization values masked; classifier uses "
                    "camera, LiDAR and Local Route only"
                ),
            }
        )
        return result


__all__ = [
    "TEMPORAL_ANCHORS_S",
    "TEMPORAL_TARGET_INDICES",
    "TemporalPlannerDataset",
]
