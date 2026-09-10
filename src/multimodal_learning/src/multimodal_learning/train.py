from __future__ import annotations

from typing import Any

import torch

from multimodal_planner_v9 import train as training
from multimodal_planner_v16_30m_candidates import train as v16_training
from multimodal_planner_v16_30m_candidates.losses import candidate_trajectory_loss
from .data import GOAL_LOOKAHEAD_M, SPATIAL_ANCHORS_M, GoalSpatialCandidateDataset
from .model import GoalSpatialCandidatePlannerV17


PATH_SCALE = torch.tensor([GOAL_LOOKAHEAD_M, 15.0], dtype=torch.float32)


def forward_batch(
    model: GoalSpatialCandidatePlannerV17, batch: dict[str, Any]
) -> dict[str, torch.Tensor]:
    return model(
        batch["front"], batch["left"], batch["right"],
        batch["lidar_bev"], batch["goal_point"],
    )


def compute_loss(outputs, batch, normalizer, weights):
    return candidate_trajectory_loss(
        outputs, batch, normalizer,
        action_weight=weights["action_weight"],
        position_weight=weights["lateral_weight"],
        step_weight=weights["lateral_step_weight"],
        acceleration_weight=weights["lateral_acceleration_weight"],
        speed_weight=weights["speed_weight"],
        speed_step_weight=weights["speed_step_weight"],
        drive_class_weight=weights["drive_class_weight"],
        stop_class_weight=weights["stop_class_weight"],
        avoid_class_weight=weights["avoid_class_weight"],
        path_scale=PATH_SCALE,
    )


def evaluate_v17(model, loader, device, amp_enabled, normalizer, weights):
    # The accumulator math is shared with V16. Temporarily bind its globals to
    # the V17 loss and station list, then expose spatially named endpoint keys.
    old_compute = v16_training.compute_loss
    old_anchors = v16_training.PATH_ANCHORS_M
    old_axis = v16_training.WAYPOINT_HORIZONS_S
    try:
        v16_training.compute_loss = compute_loss
        v16_training.PATH_ANCHORS_M = SPATIAL_ANCHORS_M
        v16_training.WAYPOINT_HORIZONS_S = SPATIAL_ANCHORS_M
        result = v16_training.evaluate_v16(
            model, loader, device, amp_enabled, normalizer, weights
        )
    finally:
        v16_training.compute_loss = old_compute
        v16_training.PATH_ANCHORS_M = old_anchors
        v16_training.WAYPOINT_HORIZONS_S = old_axis
    for state_metrics in result["state_path_metrics"].values():
        for station in SPATIAL_ANCHORS_M.tolist():
            old = f"fde_at_{station:g}s_m"
            if old in state_metrics:
                state_metrics[f"error_at_{station:g}m_m"] = state_metrics.pop(old)
    result["metric_axis"] = "fixed_route_progress_m"
    return result


def main() -> None:
    training.PlannerDataset = GoalSpatialCandidateDataset
    training.SpatialResidualSpeedPlannerV9 = GoalSpatialCandidatePlannerV17
    training.SPATIAL_ANCHORS_M = SPATIAL_ANCHORS_M
    training.forward_batch = forward_batch
    training.compute_loss = compute_loss
    training.evaluate = evaluate_v17
    # Keep the shared phase names: the trainer intentionally switches from
    # AMP to FP32 only when this function returns exactly "full".
    training.set_training_phase = v16_training.set_training_phase
    training.main()


if __name__ == "__main__":
    main()
