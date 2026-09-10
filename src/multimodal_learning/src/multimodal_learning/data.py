from __future__ import annotations

from typing import Any

import numpy as np
import torch

from multimodal_planner_v9.data import (
    ACTION_AVOID,
    ACTION_DRIVE,
    ACTION_STOP,
    FIXED_BASE_SPEED_MPS,
    PlannerDataset,
    temporal_residual_to_spatial_np,
)
from multimodal_planner_v13_goal_trajectory.data import interpolate_local_route_np


SPATIAL_ANCHORS_M = np.asarray([3.0, 6.0, 10.0, 15.0, 22.0, 30.0], dtype=np.float32)
GOAL_LOOKAHEAD_M = 30.0
GOAL_NORMALIZATION_M = np.asarray([GOAL_LOOKAHEAD_M, 15.0], dtype=np.float32)


def route_points_and_normals_np(
    normalized_route: np.ndarray,
    anchors_m: np.ndarray = SPATIAL_ANCHORS_M,
) -> tuple[np.ndarray, np.ndarray]:
    anchors = np.asarray(anchors_m, dtype=np.float32)
    points = interpolate_local_route_np(normalized_route, anchors)
    before = interpolate_local_route_np(normalized_route, np.maximum(anchors - 0.2, 0.0))
    after = interpolate_local_route_np(normalized_route, anchors + 0.2)
    tangent = after - before
    norm = np.linalg.norm(tangent, axis=1, keepdims=True).clip(1.0e-4)
    tangent /= norm
    normals = np.stack((-tangent[:, 1], tangent[:, 0]), axis=-1)
    return points.astype(np.float32), normals.astype(np.float32)


def temporal_speed_to_spatial_np(
    progress_m: np.ndarray,
    speed_mps: np.ndarray,
    anchors_m: np.ndarray = SPATIAL_ANCHORS_M,
) -> tuple[np.ndarray, np.ndarray]:
    progress = np.asarray(progress_m, dtype=np.float32).reshape(-1)
    speed = np.asarray(speed_mps, dtype=np.float32).reshape(-1)
    anchors = np.asarray(anchors_m, dtype=np.float32).reshape(-1)
    if progress.shape != speed.shape or not np.isfinite(progress).all() or not np.isfinite(speed).all():
        raise ValueError("finite progress and speed must have the same shape")
    pairs: list[tuple[float, float]] = [(0.0, float(np.clip(speed[0], 0.0, FIXED_BASE_SPEED_MPS)))]
    furthest = 0.0
    for s_value, speed_value in zip(progress.tolist(), speed.tolist()):
        if s_value < furthest - 1.0e-3 or s_value < 0.0:
            continue
        furthest = max(furthest, float(s_value))
        pair = (float(s_value), float(np.clip(speed_value, 0.0, FIXED_BASE_SPEED_MPS)))
        if abs(pair[0] - pairs[-1][0]) <= 1.0e-3:
            pairs[-1] = pair
        else:
            pairs.append(pair)
    valid = anchors <= furthest + 1.0e-3
    target = np.zeros_like(anchors, dtype=np.float32)
    if valid.any() and len(pairs) >= 2:
        source_s = np.asarray([pair[0] for pair in pairs], dtype=np.float32)
        source_v = np.asarray([pair[1] for pair in pairs], dtype=np.float32)
        target[valid] = np.interp(anchors[valid], source_s, source_v)
    return target, valid.astype(np.bool_)


class GoalSpatialCandidateDataset(PlannerDataset):
    """Fixed 3/6/10/15/22/30 m labels; Local Route is label-only."""

    def __getitem__(self, item: int) -> dict[str, Any]:
        sample = super().__getitem__(item)
        run_index, sample_index = self.lookup[item]
        run = self.runs[run_index]
        action = int(self.action_labels[item])
        route = sample["local_route"].numpy()

        base_path, normals = route_points_and_normals_np(route)
        goal_xy_m = base_path[-1].copy()
        spatial_lateral, avoid_valid_by_distance = temporal_residual_to_spatial_np(
            self.route_progress_targets[item],
            self.lateral_residual_targets[item],
            anchors_m=SPATIAL_ANCHORS_M,
        )
        avoid_path = base_path + spatial_lateral[:, None] * normals
        speed_target, speed_reached = temporal_speed_to_spatial_np(
            self.route_progress_targets[item],
            run.target[sample_index, :, 3],
        )

        drive_valid = np.zeros(6, dtype=np.bool_)
        avoid_valid = np.zeros(6, dtype=np.bool_)
        speed_valid = np.zeros(6, dtype=np.bool_)
        if action == ACTION_DRIVE:
            drive_valid[:] = True
            speed_valid[:] = speed_reached
        elif action == ACTION_AVOID:
            avoid_valid[:] = avoid_valid_by_distance
            speed_valid[:] = speed_reached
        elif action != ACTION_STOP:
            raise ValueError(f"invalid action state: {action}")

        selected_path = avoid_path if action == ACTION_AVOID else base_path
        selected_valid = avoid_valid if action == ACTION_AVOID else drive_valid
        sample["goal_point"] = torch.from_numpy(
            (goal_xy_m / GOAL_NORMALIZATION_M).astype(np.float32)
        )
        sample["target_drive_path_xy_m"] = torch.from_numpy(base_path.copy())
        sample["target_drive_path_valid"] = torch.from_numpy(drive_valid.copy())
        sample["target_avoid_path_xy_m"] = torch.from_numpy(avoid_path.astype(np.float32))
        sample["target_avoid_path_valid"] = torch.from_numpy(avoid_valid.copy())
        sample["target_speed_mps"] = torch.from_numpy(speed_target)
        sample["target_speed_valid"] = torch.from_numpy(speed_valid.copy())
        sample["target_path_xy_m"] = torch.from_numpy(selected_path.astype(np.float32))
        sample["target_path_valid"] = torch.from_numpy(selected_valid.copy())
        sample["spatial_anchors_m"] = torch.from_numpy(SPATIAL_ANCHORS_M.copy())
        sample["target_spatial_lateral_m"] = torch.from_numpy(selected_path[:, 1].astype(np.float32))
        sample["target_spatial_valid"] = torch.from_numpy(selected_valid.copy())
        sample["target_spatial_speed_delta_mps"] = torch.from_numpy(speed_target.copy())
        sample["target_spatial_speed_valid"] = torch.from_numpy(speed_valid.copy())
        for key in (
            "ego", "mgeo", "local_route", "base_speed_profile_mps",
            "base_speed_mps", "target_route_progress_m",
            "target_lateral_residual_m", "target_temporal_lateral_m",
            "target_temporal_speed_delta_mps", "target_temporal_valid",
        ):
            sample.pop(key, None)
        return sample

    def summary(self) -> dict[str, Any]:
        result = super().summary()
        for stale in (
            "valid_labels_per_anchor", "mean_valid_anchors_per_sample",
            "model_map_inputs", "lateral_target_policy",
        ):
            result.pop(stale, None)
        result.update({
            "architecture_target": "V17_fixed_spatial_30m_candidates",
            "model_inputs": ["camera3_history", "lidar_bev_history", "single_goal_point_30m"],
            "explicitly_excluded_inputs": ["current_speed", "ego", "gps", "mgeo", "local_route"],
            "spatial_anchors_m": SPATIAL_ANCHORS_M.tolist(),
            "goal_lookahead_m": GOAL_LOOKAHEAD_M,
            "path_target_policy": (
                "DRIVE=Local Route at fixed spatial stations; AVOID=recorded lateral residual "
                "projected to reached stations; STOP=classification-only"
            ),
            "speed_target_policy": "absolute speed interpolated at reached spatial stations",
            "temporal_path_target": False,
        })
        return result


__all__ = [
    "GOAL_LOOKAHEAD_M", "GOAL_NORMALIZATION_M", "SPATIAL_ANCHORS_M",
    "GoalSpatialCandidateDataset", "route_points_and_normals_np",
    "temporal_speed_to_spatial_np",
]
