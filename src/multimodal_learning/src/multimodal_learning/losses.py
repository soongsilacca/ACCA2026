import torch
import torch.nn.functional as F


LOSS_WEIGHTS = {
    "trajectory": 1.0,
    "speed": 0.4,
    "yaw": 0.2,
    "mode": 0.3,
    "signal": 0.4,
    "stop": 0.4,
    "hazard": 0.3,
    "lane": 0.3,
    "smooth": 0.05,
}


def planner_loss(output, batch):
    prediction = output["trajectories"]
    target = batch["trajectory"]
    horizon = target.shape[1]
    weights = torch.linspace(0.5, 1.5, horizon, device=target.device).view(1, 1, horizon)
    distance = torch.linalg.norm(prediction[..., :2] - target[:, None, :, :2], dim=-1)
    mode_cost = (distance * weights).mean(dim=-1)
    best_mode = mode_cost.argmin(dim=1)
    batch_index = torch.arange(target.shape[0], device=target.device)
    best = prediction[batch_index, best_mode]

    trajectory = (F.smooth_l1_loss(best[..., :2], target[..., :2], reduction="none")
                  .mean(dim=-1) * weights[:, 0]).mean()
    speed = F.smooth_l1_loss(best[..., 3], target[..., 3])
    yaw_error = best[..., 2] - target[..., 2]
    yaw_error = torch.atan2(torch.sin(yaw_error), torch.cos(yaw_error))
    yaw = F.smooth_l1_loss(yaw_error, torch.zeros_like(yaw_error))
    mode = F.cross_entropy(output["mode_logits"], best_mode)

    signal = _masked_ce(output["signal_logits"], batch.get("signal_label"))
    stop = _stop_loss(output, batch)
    hazard = _hazard_loss(output, batch)
    lane = _masked_bce(output["lane_logit"], batch.get("lane_label"))
    smooth = _smoothness_loss(best)
    terms = {
        "trajectory": trajectory, "speed": speed, "yaw": yaw, "mode": mode,
        "signal": signal, "stop": stop, "hazard": hazard, "lane": lane,
        "smooth": smooth,
    }
    terms["total"] = sum(LOSS_WEIGHTS[name] * value for name, value in terms.items())
    terms["best_mode"] = best_mode
    return terms


def trajectory_loss(output, target_or_batch):
    if isinstance(target_or_batch, dict):
        return planner_loss(output, target_or_batch)["total"]
    return planner_loss(output, {"trajectory": target_or_batch})["total"]


def _masked_ce(logits, label):
    if label is None:
        return logits.sum() * 0.0
    valid = label >= 0
    return F.cross_entropy(logits[valid], label[valid]) if valid.any() else logits.sum() * 0.0


def _masked_bce(logit, label):
    if label is None:
        return logit.sum() * 0.0
    valid = label >= 0
    return F.binary_cross_entropy_with_logits(logit[valid], label[valid]) if valid.any() else logit.sum() * 0.0


def _stop_loss(output, batch):
    label = batch.get("stop_label")
    if label is None:
        return output["stop_logit"].sum() * 0.0
    classification = F.binary_cross_entropy_with_logits(output["stop_logit"], label)
    stopped = label > 0.5
    regression = (F.smooth_l1_loss(
        output["stop_distance"][stopped], batch["stop_distance_label"][stopped]
    ) if stopped.any() else classification * 0.0)
    return classification + regression


def _hazard_loss(output, batch):
    collision = _masked_bce(output["collision_logit"], batch.get("hazard_label"))
    ttc = _masked_ce(output["ttc_logits"], batch.get("ttc_label"))
    return collision + ttc


def _smoothness_loss(trajectory):
    velocity = torch.diff(trajectory[..., :2], dim=1) / 0.2
    acceleration = torch.diff(velocity, dim=1) / 0.2
    jerk = torch.diff(acceleration, dim=1) / 0.2
    yaw_rate = torch.diff(trajectory[..., 2], dim=1) / 0.2
    return acceleration.abs().mean() + 0.2 * jerk.abs().mean() + 0.1 * yaw_rate.abs().mean()
