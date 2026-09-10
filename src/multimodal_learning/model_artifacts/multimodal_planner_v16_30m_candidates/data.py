from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import torch

from multimodal_planner_v9.data import (
    ACTION_AVOID,
    ACTION_DRIVE,
    ACTION_STOP,
    FIXED_BASE_SPEED_MPS,
    PlannerDataset,
)
from multimodal_planner_v13_goal_trajectory.data import interpolate_local_route_np


NOMINAL_SPEED_KPH = 30.0
NOMINAL_SPEED_MPS = NOMINAL_SPEED_KPH / 3.6
WAYPOINT_INDICES = np.asarray([2, 4, 7, 9, 14, 19], dtype=np.int64)
WAYPOINT_HORIZONS_S = np.asarray([0.6, 1.0, 1.6, 2.0, 3.0, 4.0], dtype=np.float32)
PATH_ANCHORS_M = WAYPOINT_HORIZONS_S * NOMINAL_SPEED_MPS
GOAL_LOOKAHEAD_M = float(NOMINAL_SPEED_MPS * 4.0)
GOAL_NORMALIZATION_M = np.asarray([GOAL_LOOKAHEAD_M, 15.0], dtype=np.float32)


def select_recorded_future_np(
    future_xy_m: np.ndarray,
    future_speed_mps: np.ndarray,
    indices: np.ndarray = WAYPOINT_INDICES,
) -> tuple[np.ndarray, np.ndarray]:
    """Select six existing 5 Hz labels without interpolation or correction."""
    xy = np.asarray(future_xy_m, dtype=np.float32)
    speed = np.asarray(future_speed_mps, dtype=np.float32).reshape(-1)
    query = np.asarray(indices, dtype=np.int64).reshape(-1)
    if xy.ndim != 2 or xy.shape[1] != 2 or xy.shape[0] != speed.shape[0]:
        raise ValueError("future XY/speed must have shapes [N,2] and [N]")
    if len(query) != 6 or query.min(initial=0) < 0 or query.max(initial=0) >= len(xy):
        raise ValueError("six waypoint indices must reference existing labels")
    if not np.isfinite(xy).all() or not np.isfinite(speed).all():
        raise ValueError("future trajectory must be finite")
    selected_xy = xy[query].copy()
    selected_speed = np.clip(speed[query], 0.0, FIXED_BASE_SPEED_MPS)
    return selected_xy.astype(np.float32), selected_speed.astype(np.float32)


class GoalCandidateDataset(PlannerDataset):
    """Single-goal, six-waypoint V16 labels.

    Local Route is used only offline to construct the single 33.33 m goal and
    the DRIVE supervision path. It is removed before the batch reaches the
    model. AVOID uses six unmodified recorded future samples.
    """

    def __getitem__(self, item: int) -> dict[str, Any]:
        sample = super().__getitem__(item)
        run_index, sample_index = self.lookup[item]
        run = self.runs[run_index]
        action = int(self.action_labels[item])
        route = sample["local_route"].numpy()

        goal_xy_m = interpolate_local_route_np(
            route, np.asarray([GOAL_LOOKAHEAD_M], dtype=np.float32)
        )[0]
        # Keep every candidate on the same temporal contract. DRIVE follows
        # the Local Route, but at the *recorded* future progress reached at
        # each of the six raw time indices. This avoids pretending every
        # DRIVE sample travels at the nominal 30 km/h used only to initialize
        # the decoder and choose the 33.33 m goal lookahead.
        drive_progress_m = self.route_progress_targets[
            item, WAYPOINT_INDICES
        ].astype(np.float32)
        drive_path = interpolate_local_route_np(route, drive_progress_m)
        raw_path = np.asarray(run.target[sample_index, :, :2], dtype=np.float32)
        raw_speed = np.asarray(run.target[sample_index, :, 3], dtype=np.float32)
        avoid_path, temporal_speed = select_recorded_future_np(raw_path, raw_speed)

        drive_valid = np.zeros(len(WAYPOINT_INDICES), dtype=np.bool_)
        avoid_valid = np.zeros(len(WAYPOINT_INDICES), dtype=np.bool_)
        speed_valid = np.zeros(len(WAYPOINT_INDICES), dtype=np.bool_)
        if action == ACTION_DRIVE:
            drive_valid[:] = True
            speed_valid[:] = True
        elif action == ACTION_AVOID:
            avoid_valid[:] = True
            speed_valid[:] = True
        elif action != ACTION_STOP:
            raise ValueError(f"invalid action state: {action}")

        selected_path = avoid_path if action == ACTION_AVOID else drive_path
        selected_valid = avoid_valid if action == ACTION_AVOID else drive_valid

        sample["goal_point"] = torch.from_numpy(
            (goal_xy_m / GOAL_NORMALIZATION_M).astype(np.float32)
        )
        sample["target_drive_path_xy_m"] = torch.from_numpy(drive_path)
        sample["target_drive_path_valid"] = torch.from_numpy(drive_valid.copy())
        sample["target_avoid_path_xy_m"] = torch.from_numpy(avoid_path)
        sample["target_avoid_path_valid"] = torch.from_numpy(avoid_valid.copy())
        sample["target_speed_mps"] = torch.from_numpy(temporal_speed)
        sample["target_speed_valid"] = torch.from_numpy(speed_valid.copy())
        sample["target_path_xy_m"] = torch.from_numpy(selected_path.copy())
        sample["target_path_valid"] = torch.from_numpy(selected_valid.copy())
        sample["path_anchors_m"] = torch.from_numpy(PATH_ANCHORS_M.copy())
        sample["waypoint_horizons_s"] = torch.from_numpy(WAYPOINT_HORIZONS_S.copy())
        sample["target_drive_progress_m"] = torch.from_numpy(
            drive_progress_m.copy()
        )

        # Compatibility-only labels required by the shared training harness.
        sample["target_spatial_lateral_m"] = torch.from_numpy(selected_path[:, 1].copy())
        sample["target_spatial_valid"] = torch.from_numpy(selected_valid.copy())
        sample["target_spatial_speed_delta_mps"] = torch.from_numpy(temporal_speed.copy())
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
            "spatial_anchors_m", "valid_labels_per_anchor",
            "mean_valid_anchors_per_sample", "lateral_target_policy",
            "model_map_inputs",
        ):
            result.pop(stale, None)
        result.update(
            {
                "architecture_target": "V16_single_33m_goal_tcp6_candidates",
                "model_inputs": [
                    "front_camera_history", "left_camera_history",
                    "right_camera_history", "lidar_bev_history",
                    "single_ego_relative_goal_point_33.33m",
                ],
                "explicitly_excluded_inputs": [
                    "current_speed", "ego_state", "imu", "gps",
                    "gps_health", "mgeo", "local_route_64x4",
                    "intermediate_route_points", "route_command",
                ],
                "goal_lookahead_m": GOAL_LOOKAHEAD_M,
                "waypoint_horizons_s": WAYPOINT_HORIZONS_S.tolist(),
                "nominal_path_anchors_m_at_30kph": PATH_ANCHORS_M.tolist(),
                "raw_target_indices": WAYPOINT_INDICES.tolist(),
                "candidate_policy": (
                    "DRIVE and AVOID candidates are always emitted; downstream "
                    "state machine selects; STOP is classification-only"
                ),
                "path_target_policy": (
                    "DRIVE=Local Route sampled offline at recorded future "
                    "progress for raw indices 2/4/7/9/14/19; AVOID=existing "
                    "raw future XY at the same indices; no trajectory repair"
                ),
                "speed_target_policy": (
                    "absolute recorded future speed at the same six raw frames; "
                    "STOP is masked"
                ),
            }
        )
        return result


__all__ = [
    "GOAL_LOOKAHEAD_M", "GOAL_NORMALIZATION_M", "NOMINAL_SPEED_KPH",
    "NOMINAL_SPEED_MPS", "PATH_ANCHORS_M", "WAYPOINT_HORIZONS_S",
    "WAYPOINT_INDICES", "GoalCandidateDataset", "select_recorded_future_np",
]
