from __future__ import annotations

import torch
import torch.nn.functional as F

from multimodal_planner_v7.model import HORIZON_DT_SEC, SPEED_SCALE_MPS


def _weighted_mean(
    value: torch.Tensor,
    sample_weight: torch.Tensor,
    normalizer: float,
) -> torch.Tensor:
    return (value * sample_weight).mean() / max(float(normalizer), 1.0e-8)


def planner_loss(
    outputs: dict[str, torch.Tensor],
    target: torch.Tensor,
    target_route_progress_m: torch.Tensor,
    target_lateral_residual_m: torch.Tensor,
    motion_state: torch.Tensor,
    sample_weight: torch.Tensor,
    sample_weight_normalizer: float = 1.0,
    *,
    speed_weight: float = 1.0,
    acceleration_weight: float = 0.25,
    jerk_weight: float = 0.1,
    progress_weight: float = 0.25,
    lateral_weight: float = 1.0,
    lateral_step_weight: float = 0.5,
    lateral_acceleration_weight: float = 0.25,
    state_weight: float = 0.2,
    speed_delta_regularization_weight: float = 0.01,
    stop_class_weight: float = 4.0,
    drive_class_weight: float = 1.0,
) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
    if target.ndim != 3 or target.shape[-2:] != (20, 4):
        raise ValueError(f"target must have shape [batch,20,4], got {target.shape}")
    prediction = outputs["future_speed"]
    target_speed = target[..., 3]
    if prediction.shape != target_speed.shape:
        raise ValueError(
            f"future_speed must have shape {target_speed.shape}, "
            f"got {prediction.shape}"
        )

    speed_per_sample = F.smooth_l1_loss(
        prediction,
        target_speed,
        reduction="none",
    ).mean(dim=1)
    pred_acc = (prediction[:, 1:] - prediction[:, :-1]) / HORIZON_DT_SEC
    target_acc = (target_speed[:, 1:] - target_speed[:, :-1]) / HORIZON_DT_SEC
    acceleration_per_sample = F.smooth_l1_loss(
        pred_acc,
        target_acc,
        reduction="none",
    ).mean(dim=1)
    pred_jerk = (pred_acc[:, 1:] - pred_acc[:, :-1]) / HORIZON_DT_SEC
    target_jerk = (target_acc[:, 1:] - target_acc[:, :-1]) / HORIZON_DT_SEC
    jerk_per_sample = F.smooth_l1_loss(
        pred_jerk,
        target_jerk,
        reduction="none",
    ).mean(dim=1)

    target_xy_m = target[..., :2] * 50.0
    target_progress_m = target_route_progress_m
    target_lateral_m = target_lateral_residual_m
    progress_per_sample = F.smooth_l1_loss(
        outputs["forward_progress_m"] / 50.0,
        target_progress_m / 50.0,
        reduction="none",
    ).mean(dim=1)
    predicted_lateral_m = outputs["lateral_residual_m"]
    lateral_per_sample = F.smooth_l1_loss(
        predicted_lateral_m / 5.0,
        target_lateral_m / 5.0,
        reduction="none",
    ).mean(dim=1)
    pred_lateral_step = predicted_lateral_m[:, 1:] - predicted_lateral_m[:, :-1]
    target_lateral_step = target_lateral_m[:, 1:] - target_lateral_m[:, :-1]
    lateral_step_per_sample = F.smooth_l1_loss(
        pred_lateral_step / 5.0,
        target_lateral_step / 5.0,
        reduction="none",
    ).mean(dim=1)
    pred_lateral_acc = pred_lateral_step[:, 1:] - pred_lateral_step[:, :-1]
    target_lateral_acc = target_lateral_step[:, 1:] - target_lateral_step[:, :-1]
    lateral_acceleration_per_sample = F.smooth_l1_loss(
        pred_lateral_acc / 5.0,
        target_lateral_acc / 5.0,
        reduction="none",
    ).mean(dim=1)

    speed = _weighted_mean(
        speed_per_sample,
        sample_weight,
        sample_weight_normalizer,
    )
    acceleration = _weighted_mean(
        acceleration_per_sample,
        sample_weight,
        sample_weight_normalizer,
    )
    jerk = _weighted_mean(
        jerk_per_sample,
        sample_weight,
        sample_weight_normalizer,
    )
    progress = _weighted_mean(
        progress_per_sample,
        sample_weight,
        sample_weight_normalizer,
    )
    lateral = _weighted_mean(
        lateral_per_sample,
        sample_weight,
        sample_weight_normalizer,
    )
    lateral_step = _weighted_mean(
        lateral_step_per_sample,
        sample_weight,
        sample_weight_normalizer,
    )
    lateral_acceleration = _weighted_mean(
        lateral_acceleration_per_sample,
        sample_weight,
        sample_weight_normalizer,
    )

    logits = outputs["motion_state_logits"]
    class_weights = torch.as_tensor(
        [stop_class_weight, drive_class_weight],
        dtype=logits.dtype,
        device=logits.device,
    )
    state = F.cross_entropy(
        logits,
        motion_state,
        weight=class_weights,
    )
    speed_delta_regularization = outputs["speed_delta"].square().mean()
    total = (
        speed_weight * speed
        + acceleration_weight * acceleration
        + jerk_weight * jerk
        + progress_weight * progress
        + lateral_weight * lateral
        + lateral_step_weight * lateral_step
        + lateral_acceleration_weight * lateral_acceleration
        + state_weight * state
        + speed_delta_regularization_weight * speed_delta_regularization
    )
    terms = {
        "loss": total.detach(),
        "speed": speed.detach(),
        "acceleration": acceleration.detach(),
        "jerk": jerk.detach(),
        "progress": progress.detach(),
        "lateral": lateral.detach(),
        "lateral_step": lateral_step.detach(),
        "lateral_acceleration": lateral_acceleration.detach(),
        "state": state.detach(),
        "speed_delta_regularization": speed_delta_regularization.detach(),
        "speed_mae_mps": (
            torch.mean(torch.abs(prediction - target_speed)) * SPEED_SCALE_MPS
        ).detach(),
    }
    return total, terms


__all__ = ["planner_loss"]
