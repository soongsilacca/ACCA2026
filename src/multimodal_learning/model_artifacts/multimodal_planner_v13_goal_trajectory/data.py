from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import torch

from multimodal_planner_v9.data import (
    ACTION_DRIVE,
    ACTION_STOP,
    FIXED_BASE_SPEED_MPS,
)
from multimodal_planner_v9.model import XY_SCALE_M
from multimodal_planner_v10.data import (
    TEMPORAL_ANCHORS_S,
    TEMPORAL_TARGET_INDICES,
    TemporalPlannerDataset,
)


GOAL_LOOKAHEAD_M = 60.0


def interpolate_local_route_np(
    normalized_route: np.ndarray, progress_m: np.ndarray
) -> np.ndarray:
    points = np.asarray(normalized_route[:, :2], dtype=np.float32) * XY_SCALE_M
    segments = points[1:] - points[:-1]
    lengths = np.linalg.norm(segments, axis=1).clip(1.0e-4)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    starts = points[:-1]
    fractions = np.clip(
        -(starts * segments).sum(axis=1) / np.square(lengths), 0.0, 1.0
    )
    projected = starts + fractions[:, None] * segments
    closest = int(np.square(projected).sum(axis=1).argmin())
    origin_s = cumulative[closest] + fractions[closest] * lengths[closest]
    query = origin_s + np.maximum(np.asarray(progress_m, dtype=np.float32), 0.0)
    clipped = np.minimum(query, cumulative[-1])
    indices = np.searchsorted(cumulative, clipped, side="right") - 1
    indices = np.clip(indices, 0, len(segments) - 1)
    local_fraction = np.clip(
        (clipped - cumulative[indices]) / lengths[indices], 0.0, 1.0
    )
    result = points[indices] + local_fraction[:, None] * segments[indices]
    overflow = np.maximum(query - cumulative[-1], 0.0)
    result += overflow[:, None] * segments[-1] / lengths[-1]
    return result.astype(np.float32)


class GoalTrajectoryDataset(TemporalPlannerDataset):
    """Six direct XY/speed targets V13; route is consumed only to build 30m goal labels."""

    def __getitem__(self, item: int) -> dict[str, Any]:
        sample = super().__getitem__(item)
        run_index, sample_index = self.lookup[item]
        run = self.runs[run_index]
        action = int(self.action_labels[item])
        route = sample["local_route"].numpy()

        goal_xy_m = interpolate_local_route_np(
            route, np.asarray([GOAL_LOOKAHEAD_M], dtype=np.float32)
        )[0]
        raw_path = np.asarray(
            run.target[sample_index, TEMPORAL_TARGET_INDICES, :2],
            dtype=np.float32,
        )
        if action == ACTION_DRIVE:
            progress = self.route_progress_targets[
                item, TEMPORAL_TARGET_INDICES
            ].astype(np.float32)
            path_target = interpolate_local_route_np(route, progress)
        else:
            path_target = raw_path.copy()

        speed_target = np.clip(
            run.target[sample_index, TEMPORAL_TARGET_INDICES, 3],
            0.0,
            FIXED_BASE_SPEED_MPS,
        ).astype(np.float32)
        valid = np.ones(6, dtype=np.bool_)
        if action == ACTION_STOP:
            valid[:] = False

        sample["goal_point"] = torch.from_numpy(goal_xy_m / GOAL_LOOKAHEAD_M)
        sample["target_path_xy_m"] = torch.from_numpy(path_target)
        sample["target_speed_mps"] = torch.from_numpy(speed_target)
        sample["target_path_valid"] = torch.from_numpy(valid.copy())
        sample["target_speed_valid"] = torch.from_numpy(valid.copy())
        # Compatibility metrics use Y and absolute speed, never route input.
        sample["target_spatial_lateral_m"] = torch.from_numpy(path_target[:, 1])
        sample["target_spatial_valid"] = torch.from_numpy(valid.copy())
        sample["target_spatial_speed_delta_mps"] = torch.from_numpy(speed_target)
        sample["target_spatial_speed_valid"] = torch.from_numpy(valid.copy())
        for key in (
            "ego", "local_route", "base_speed_profile_mps", "base_speed_mps",
            "target_route_progress_m", "target_lateral_residual_m",
            "target_temporal_lateral_m", "target_temporal_speed_delta_mps",
            "target_temporal_valid",
        ):
            sample.pop(key, None)
        return sample

    def summary(self) -> dict[str, Any]:
        result = super().summary()
        result.update(
            {
                "architecture_target": "V13_goal_point_direct_6xXY_plus_speed_beta",
                "model_inputs": [
                    "front_camera_history", "left_camera_history",
                    "right_camera_history", "lidar_bev_history",
                    "ego_relative_goal_point_xy",
                ],
                "explicitly_excluded_inputs": [
                    "current_speed", "ego_state", "imu", "gps",
                    "gps_health", "mgeo", "local_route_64x4",
                ],
                "goal_lookahead_m": GOAL_LOOKAHEAD_M,
                "path_target_policy": (
                    "DRIVE=route-derived label only; AVOID=recorded XY; "
                    "STOP=classification only"
                ),
                "speed_target_policy": (
                    "absolute future speed for DRIVE/AVOID; STOP classification only"
                ),
            }
        )
        for stale in (
            "classifier_ego_policy", "lateral_target_policy",
            "model_map_inputs", "speed_target_policy",
        ):
            if stale == "speed_target_policy":
                continue
            result.pop(stale, None)
        return result


__all__ = [
    "GOAL_LOOKAHEAD_M", "GoalTrajectoryDataset",
    "TEMPORAL_ANCHORS_S", "TEMPORAL_TARGET_INDICES",
    "interpolate_local_route_np",
]
