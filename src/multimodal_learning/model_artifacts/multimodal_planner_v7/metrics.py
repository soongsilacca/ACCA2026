from __future__ import annotations

from typing import Any

import torch

from multimodal_planner_v6.metrics import (
    MotionStateMetricAccumulator,
    TrajectoryMetricAccumulator,
    trajectory_diagnostics,
)
from multimodal_planner_v7.model import HORIZON_DT_SEC, SPEED_SCALE_MPS


class SpeedMetricAccumulator:
    def __init__(self) -> None:
        self.count = 0
        self.speed_abs_sum = 0.0
        self.speed_sq_sum = 0.0
        self.acc_abs_sum = 0.0
        self.final_speed_abs_sum = 0.0
        self.progress_abs_sum = 0.0
        self.lateral_abs_sum = 0.0
        self.avoidance_lateral_abs_sum = 0.0
        self.avoidance_count = 0

    def update(
        self,
        outputs: dict[str, torch.Tensor],
        target: torch.Tensor,
        target_lateral_residual_m: torch.Tensor,
        avoidance: torch.Tensor,
    ) -> None:
        prediction = outputs["future_speed"].detach().float()
        truth = target[..., 3].detach().float()
        speed_error_mps = (prediction - truth) * SPEED_SCALE_MPS
        pred_acc = (
            (prediction[:, 1:] - prediction[:, :-1])
            * SPEED_SCALE_MPS
            / HORIZON_DT_SEC
        )
        truth_acc = (
            (truth[:, 1:] - truth[:, :-1])
            * SPEED_SCALE_MPS
            / HORIZON_DT_SEC
        )
        target_xy_m = target[..., :2].detach().float() * 50.0
        target_xy = torch.cat(
            (torch.zeros_like(target_xy_m[:, :1]), target_xy_m),
            dim=1,
        )
        target_progress = torch.linalg.vector_norm(
            target_xy[:, 1:] - target_xy[:, :-1],
            dim=-1,
        ).sum(dim=1)
        predicted_progress = outputs["forward_progress_m"][:, -1].detach().float()
        lateral_error = (
            outputs["lateral_residual_m"].detach().float()
            - target_lateral_residual_m.detach().float()
        ).abs().mean(dim=1)
        avoidance = avoidance.detach().bool()

        batch = prediction.shape[0]
        self.count += batch
        self.speed_abs_sum += float(speed_error_mps.abs().mean(dim=1).sum())
        self.speed_sq_sum += float(speed_error_mps.square().mean(dim=1).sum())
        self.acc_abs_sum += float((pred_acc - truth_acc).abs().mean(dim=1).sum())
        self.final_speed_abs_sum += float(speed_error_mps[:, -1].abs().sum())
        self.progress_abs_sum += float(
            (predicted_progress - target_progress).abs().sum()
        )
        self.lateral_abs_sum += float(lateral_error.sum())
        self.avoidance_lateral_abs_sum += float(lateral_error[avoidance].sum())
        self.avoidance_count += int(avoidance.sum())

    def result(self) -> dict[str, Any]:
        count = max(self.count, 1)
        return {
            "count": self.count,
            "speed_mae_mps": self.speed_abs_sum / count,
            "speed_rmse_mps": (self.speed_sq_sum / count) ** 0.5,
            "acceleration_mae_mps2": self.acc_abs_sum / count,
            "final_speed_mae_mps": self.final_speed_abs_sum / count,
            "final_progress_mae_m": self.progress_abs_sum / count,
            "route_residual_mae_m": self.lateral_abs_sum / count,
            "avoidance_count": self.avoidance_count,
            "avoidance_route_residual_mae_m": (
                self.avoidance_lateral_abs_sum / max(self.avoidance_count, 1)
            ),
        }


__all__ = [
    "MotionStateMetricAccumulator",
    "SpeedMetricAccumulator",
    "TrajectoryMetricAccumulator",
    "trajectory_diagnostics",
]
