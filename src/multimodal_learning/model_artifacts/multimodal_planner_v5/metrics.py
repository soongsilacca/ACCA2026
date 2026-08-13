from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import torch


XY_SCALE_M = 50.0
YAW_SCALE_RAD = math.pi
HORIZON_STEPS = {1: 5, 2: 10, 4: 20}
LATERAL_FLIP_DEADBAND_M = 0.1


def _gt_unit_normal(target_xy: torch.Tensor) -> torch.Tensor:
    tangent = torch.empty_like(target_xy)
    tangent[:, 0] = target_xy[:, 1] - target_xy[:, 0]
    tangent[:, -1] = target_xy[:, -1] - target_xy[:, -2]
    tangent[:, 1:-1] = target_xy[:, 2:] - target_xy[:, :-2]
    tangent_norm = torch.linalg.vector_norm(tangent, dim=-1, keepdim=True)
    fallback = torch.zeros_like(tangent)
    fallback[..., 0] = 1.0
    unit_tangent = torch.where(
        tangent_norm > 1e-6,
        tangent / tangent_norm.clamp_min(1e-6),
        fallback,
    )
    return torch.stack((-unit_tangent[..., 1], unit_tangent[..., 0]), dim=-1)


def trajectory_diagnostics(
    outputs: dict[str, torch.Tensor],
    target: torch.Tensor,
) -> dict[str, torch.Tensor]:
    """Return physical-unit single-trajectory V5 boundary-aware metrics."""
    prediction = outputs["trajectory"].float()
    target_value = target.float()
    if prediction.ndim != 3 or prediction.shape[-2:] != (20, 4):
        raise ValueError(
            "trajectory prediction must have shape [batch, 20, 4], "
            f"got {tuple(prediction.shape)}"
        )
    if target_value.shape != prediction.shape:
        raise ValueError(
            "target must match V5 prediction shape [batch, 20, 4], "
            f"got {tuple(target_value.shape)}"
        )

    xy_delta_m = (prediction[..., :2] - target_value[..., :2]) * XY_SCALE_M
    displacement_error_m = torch.linalg.vector_norm(xy_delta_m, dim=-1)
    yaw_delta_rad = (prediction[..., 2] - target_value[..., 2]) * YAW_SCALE_RAD
    yaw_error_deg = torch.rad2deg(
        torch.atan2(torch.sin(yaw_delta_rad), torch.cos(yaw_delta_rad)).abs()
    )

    origin_xy = torch.zeros_like(prediction[:, :1, :2])
    prediction_with_origin = torch.cat((origin_xy, prediction[..., :2]), dim=1)
    target_with_origin = torch.cat((origin_xy, target_value[..., :2]), dim=1)
    prediction_step = (
        prediction_with_origin[:, 1:] - prediction_with_origin[:, :-1]
    )
    target_step = target_with_origin[:, 1:] - target_with_origin[:, :-1]
    step_error_m = torch.linalg.vector_norm(
        (prediction_step - target_step) * XY_SCALE_M, dim=-1
    )
    prediction_acc = prediction_step[:, 1:] - prediction_step[:, :-1]
    target_acc = target_step[:, 1:] - target_step[:, :-1]
    acceleration_error_m = torch.linalg.vector_norm(
        (prediction_acc - target_acc) * XY_SCALE_M, dim=-1
    )

    prediction_path_yaw = torch.atan2(
        prediction_step[..., 1], prediction_step[..., 0]
    )
    origin_yaw = torch.zeros_like(prediction[:, :1, 2])
    prediction_segment_yaw = torch.cat(
        (origin_yaw, prediction[:, :-1, 2]), dim=1
    )
    prediction_yaw = prediction_segment_yaw * YAW_SCALE_RAD
    heading_delta = torch.atan2(
        torch.sin(prediction_yaw - prediction_path_yaw),
        torch.cos(prediction_yaw - prediction_path_yaw),
    ).abs()
    moving = torch.linalg.vector_norm(target_step, dim=-1) > 1e-4
    heading_consistency_mae_deg = (
        (torch.rad2deg(heading_delta) * moving).sum(dim=-1)
        / moving.sum(dim=-1).clamp_min(1)
    )

    unit_normal = _gt_unit_normal(target_value[..., :2])
    lateral_residual_m = (xy_delta_m * unit_normal).sum(dim=-1)
    previous = lateral_residual_m[:, :-1]
    current = lateral_residual_m[:, 1:]
    outside_deadband = (
        (previous.abs() > LATERAL_FLIP_DEADBAND_M)
        & (current.abs() > LATERAL_FLIP_DEADBAND_M)
    )
    sign_flips = outside_deadband & ((previous * current) < 0.0)
    flip_count = sign_flips.sum(dim=-1).float()
    flip_rate = flip_count / outside_deadband.sum(dim=-1).clamp_min(1)

    first_prediction_m = prediction[:, 0, :2] * XY_SCALE_M
    first_target_m = target_value[:, 0, :2] * XY_SCALE_M
    first_target_norm_m = torch.linalg.vector_norm(first_target_m, dim=-1)
    first_direction_dot = (first_prediction_m * first_target_m).sum(dim=-1)
    origin_direction_mismatch = torch.where(
        first_target_norm_m > 0.05,
        first_direction_dot < 0.0,
        first_prediction_m[:, 0] < -0.05,
    ).float()

    diagnostics: dict[str, torch.Tensor] = {
        "longitudinal_mae_m": xy_delta_m[..., 0].abs().mean(dim=-1),
        "lateral_mae_m": xy_delta_m[..., 1].abs().mean(dim=-1),
        "yaw_mae_deg": yaw_error_deg.mean(dim=-1),
        "step_mae_m": step_error_m.mean(dim=-1),
        "acceleration_mae_m": acceleration_error_m.mean(dim=-1),
        "heading_consistency_mae_deg": heading_consistency_mae_deg,
        "first_waypoint_error_m": displacement_error_m[:, 0],
        "predicted_first_x_m": first_prediction_m[:, 0],
        "target_first_x_m": first_target_m[:, 0],
        "origin_direction_mismatch": origin_direction_mismatch,
        "gt_relative_lateral_sign_flip_count": flip_count,
        "gt_relative_lateral_sign_flip_rate": flip_rate,
    }
    for seconds, steps in HORIZON_STEPS.items():
        diagnostics[f"ade_{seconds}s_m"] = displacement_error_m[:, :steps].mean(
            dim=-1
        )
        diagnostics[f"fde_{seconds}s_m"] = displacement_error_m[:, steps - 1]

    diagnostics["ade_m"] = diagnostics["ade_4s_m"]
    diagnostics["fde_m"] = diagnostics["fde_4s_m"]
    return diagnostics


SCALAR_KEYS = (
    "longitudinal_mae_m",
    "lateral_mae_m",
    "yaw_mae_deg",
    "step_mae_m",
    "acceleration_mae_m",
    "heading_consistency_mae_deg",
    "first_waypoint_error_m",
    "predicted_first_x_m",
    "target_first_x_m",
    "origin_direction_mismatch",
    "gt_relative_lateral_sign_flip_count",
    "gt_relative_lateral_sign_flip_rate",
    "ade_1s_m",
    "fde_1s_m",
    "ade_2s_m",
    "fde_2s_m",
    "ade_4s_m",
    "fde_4s_m",
)


@dataclass
class _GroupTotals:
    count: int = 0
    scalar_sums: dict[str, float] = field(default_factory=dict)

    def update(
        self,
        diagnostics: dict[str, torch.Tensor],
        mask: torch.Tensor,
    ) -> None:
        mask = mask.bool()
        added = int(mask.sum().item())
        if added == 0:
            return
        self.count += added
        for key in SCALAR_KEYS:
            values = diagnostics[key][mask].detach().float()
            self.scalar_sums[key] = self.scalar_sums.get(key, 0.0) + float(
                values.sum().item()
            )

    def result(self) -> dict[str, Any]:
        if self.count == 0:
            return {"count": 0, "trajectory": None}
        mean = {key: self.scalar_sums[key] / self.count for key in SCALAR_KEYS}
        return {
            "count": self.count,
            "trajectory": {
                "ade_m": {
                    f"{seconds}s": mean[f"ade_{seconds}s_m"]
                    for seconds in HORIZON_STEPS
                },
                "fde_m": {
                    f"{seconds}s": mean[f"fde_{seconds}s_m"]
                    for seconds in HORIZON_STEPS
                },
                "longitudinal_mae_m": mean["longitudinal_mae_m"],
                "lateral_mae_m": mean["lateral_mae_m"],
                "yaw_mae_deg": mean["yaw_mae_deg"],
                "step_mae_m": mean["step_mae_m"],
                "acceleration_mae_m": mean["acceleration_mae_m"],
                "heading_consistency_mae_deg": mean[
                    "heading_consistency_mae_deg"
                ],
                "first_waypoint_error_m": mean["first_waypoint_error_m"],
                "predicted_first_x_m": mean["predicted_first_x_m"],
                "target_first_x_m": mean["target_first_x_m"],
                "origin_direction_mismatch_rate": mean[
                    "origin_direction_mismatch"
                ],
                "gt_relative_lateral_sign_flip_count": mean[
                    "gt_relative_lateral_sign_flip_count"
                ],
                "gt_relative_lateral_sign_flip_rate": mean[
                    "gt_relative_lateral_sign_flip_rate"
                ],
            },
        }


class TrajectoryMetricAccumulator:
    """Aggregate V5 metrics for all/GPS-blackout/non-blackout groups."""

    def __init__(self) -> None:
        self.groups = {
            "all": _GroupTotals(),
            "blackout": _GroupTotals(),
            "non_blackout": _GroupTotals(),
        }

    def update(
        self,
        diagnostics: dict[str, torch.Tensor],
        gps_blackout: torch.Tensor,
    ) -> None:
        blackout = gps_blackout.bool()
        all_samples = torch.ones_like(blackout, dtype=torch.bool)
        self.groups["all"].update(diagnostics, all_samples)
        self.groups["blackout"].update(diagnostics, blackout)
        self.groups["non_blackout"].update(diagnostics, ~blackout)

    def result(self) -> dict[str, Any]:
        return {name: group.result() for name, group in self.groups.items()}
