from __future__ import annotations

import torch
import torch.nn.functional as F


def _masked_sample_mean(
    value: torch.Tensor,
    mask: torch.Tensor,
) -> torch.Tensor:
    count = mask.sum(dim=1).clamp_min(1)
    return (value * mask).sum(dim=1) / count


def _weighted_mean(
    value: torch.Tensor,
    sample_weight: torch.Tensor,
    normalizer: float,
) -> torch.Tensor:
    return (value * sample_weight).mean() / max(float(normalizer), 1.0e-8)


def planner_loss(
    outputs: dict[str, torch.Tensor],
    target_lateral_m: torch.Tensor,
    target_valid: torch.Tensor,
    motion_state: torch.Tensor,
    sample_weight: torch.Tensor,
    sample_weight_normalizer: float = 1.0,
    *,
    lateral_weight: float = 1.0,
    lateral_step_weight: float = 0.5,
    lateral_acceleration_weight: float = 0.25,
    unobserved_prior_weight: float = 0.01,
    state_weight: float = 0.2,
    stop_class_weight: float = 4.0,
    drive_class_weight: float = 1.0,
) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
    prediction = outputs["lateral_residual_m"]
    if prediction.shape != target_lateral_m.shape:
        raise ValueError(
            f"prediction and target shape mismatch: "
            f"{prediction.shape} != {target_lateral_m.shape}"
        )
    valid = target_valid.to(dtype=prediction.dtype)
    lateral_values = F.smooth_l1_loss(
        prediction / 5.0,
        target_lateral_m / 5.0,
        reduction="none",
    )
    lateral_per_sample = _masked_sample_mean(lateral_values, valid)

    pred_step = prediction[:, 1:] - prediction[:, :-1]
    target_step = target_lateral_m[:, 1:] - target_lateral_m[:, :-1]
    step_valid = valid[:, 1:] * valid[:, :-1]
    step_values = F.smooth_l1_loss(
        pred_step / 5.0,
        target_step / 5.0,
        reduction="none",
    )
    step_per_sample = _masked_sample_mean(step_values, step_valid)

    pred_acc = pred_step[:, 1:] - pred_step[:, :-1]
    target_acc = target_step[:, 1:] - target_step[:, :-1]
    acc_valid = step_valid[:, 1:] * step_valid[:, :-1]
    acc_values = F.smooth_l1_loss(
        pred_acc / 5.0,
        target_acc / 5.0,
        reduction="none",
    )
    acceleration_per_sample = _masked_sample_mean(acc_values, acc_valid)

    invalid = 1.0 - valid
    invalid_count = invalid.sum(dim=1).clamp_min(1.0)
    unobserved_prior_per_sample = (
        prediction.square() * invalid
    ).sum(dim=1) / invalid_count

    lateral = _weighted_mean(
        lateral_per_sample, sample_weight, sample_weight_normalizer
    )
    lateral_step = _weighted_mean(
        step_per_sample, sample_weight, sample_weight_normalizer
    )
    lateral_acceleration = _weighted_mean(
        acceleration_per_sample, sample_weight, sample_weight_normalizer
    )
    unobserved_prior = _weighted_mean(
        unobserved_prior_per_sample,
        sample_weight,
        sample_weight_normalizer,
    )
    class_weights = torch.as_tensor(
        [stop_class_weight, drive_class_weight],
        dtype=prediction.dtype,
        device=prediction.device,
    )
    state = F.cross_entropy(
        outputs["motion_state_logits"],
        motion_state,
        weight=class_weights,
    )
    total = (
        lateral_weight * lateral
        + lateral_step_weight * lateral_step
        + lateral_acceleration_weight * lateral_acceleration
        + unobserved_prior_weight * unobserved_prior
        + state_weight * state
    )
    valid_count = valid.sum()
    lateral_mae_m = (
        ((prediction - target_lateral_m).abs() * valid).sum()
        / valid_count.clamp_min(1.0)
    )
    terms = {
        "loss": total.detach(),
        "lateral": lateral.detach(),
        "lateral_step": lateral_step.detach(),
        "lateral_acceleration": lateral_acceleration.detach(),
        "unobserved_prior": unobserved_prior.detach(),
        "state": state.detach(),
        "lateral_mae_m": lateral_mae_m.detach(),
        "valid_anchor_fraction": valid.mean().detach(),
    }
    return total, terms


__all__ = ["planner_loss"]

