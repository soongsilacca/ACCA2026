from __future__ import annotations

from typing import Any

import torch
import torch.nn as nn

from multimodal_planner_v9.data import ACTION_COUNT, FIXED_BASE_SPEED_MPS
from multimodal_planner_v9.model import ModelConfig
from multimodal_planner_v10.model import TemporalResidualPlannerV10


PATH_X_MIN_M = -10.0
PATH_X_MAX_M = 60.0
PATH_Y_MIN_M = -20.0
PATH_Y_MAX_M = 20.0
BETA_MIN_SHAPE = 1.01
BETA_MAX_SHAPE = 100.0


def _beta_parameters(raw: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    raw = raw.float().reshape(*raw.shape[:-1], -1, 2)
    shapes = (torch.nn.functional.softplus(raw) + BETA_MIN_SHAPE).clamp_max(
        BETA_MAX_SHAPE
    )
    return shapes[..., 0], shapes[..., 1]


def _scale_beta_mean(
    alpha: torch.Tensor, beta: torch.Tensor, low: float, high: float
) -> torch.Tensor:
    unit = alpha / (alpha + beta).clamp_min(1.0e-6)
    return low + unit * (high - low)


class GoalTrajectoryPlannerV13(TemporalResidualPlannerV10):
    """Camera + LiDAR + 30m goal point; direct trajectory planner V13."""

    architecture_name = "multimodal_planner_v13_goal_direct_trajectory"

    def __init__(self, config: ModelConfig | None = None) -> None:
        super().__init__(config)
        dim = self.config.hidden_dim
        dropout = self.config.dropout
        del self.route_encoder
        del self.regression_ego_adapter
        del self.lateral_head
        del self.speed_delta_head
        self.goal_encoder = nn.Sequential(
            nn.LayerNorm(2), nn.Linear(2, dim), nn.GELU(),
            nn.Dropout(dropout), nn.Linear(dim, dim), nn.LayerNorm(dim),
        )
        self.path_beta_head = nn.Sequential(
            nn.LayerNorm(dim), nn.Linear(dim, dim * 2), nn.GELU(),
            nn.Dropout(dropout), nn.Linear(dim * 2, 24),
        )
        self.speed_beta_head = nn.Sequential(
            nn.LayerNorm(dim), nn.Linear(dim, dim * 2), nn.GELU(),
            nn.Dropout(dropout), nn.Linear(dim * 2, 12),
        )
        # Compatibility names used only by the shared freeze schedule.
        self.lateral_head = self.path_beta_head
        self.speed_delta_head = self.speed_beta_head
        nn.init.zeros_(self.path_beta_head[-1].weight)
        nn.init.zeros_(self.path_beta_head[-1].bias)
        nn.init.zeros_(self.speed_beta_head[-1].weight)
        nn.init.zeros_(self.speed_beta_head[-1].bias)

    def _goal_context(
        self,
        front: torch.Tensor,
        left: torch.Tensor,
        right: torch.Tensor,
        lidar_bev: torch.Tensor,
        goal_point: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        if goal_point.ndim != 2 or goal_point.shape[1] != 2:
            raise ValueError("goal_point must have shape [batch,2]")
        batch, history = front.shape[:2]
        no_ego = torch.zeros(
            batch, history, 16, device=front.device, dtype=front.dtype
        )
        frame_slots = self.encode_frames(front, left, right, lidar_bev, no_ego)
        _, _, slots, dim = frame_slots.shape
        temporal_input = frame_slots.permute(0, 2, 1, 3).reshape(
            batch * slots, history, dim
        )
        _, hidden = self.temporal_gru(temporal_input)
        fused = hidden[-1].reshape(batch, slots, dim)
        goal_token = self.goal_encoder(goal_point).unsqueeze(1)
        for layer in self.route_fusion:
            fused = layer(fused, goal_token)
        fused = self.output_norm(fused)
        return fused.mean(dim=1), fused

    def forward(
        self,
        front: torch.Tensor,
        left: torch.Tensor,
        right: torch.Tensor,
        lidar_bev: torch.Tensor,
        goal_point: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        context, fused = self._goal_context(
            front, left, right, lidar_bev, goal_point
        )
        path_raw = self.path_beta_head(context).reshape(-1, 6, 2, 2)
        path_alpha, path_beta = _beta_parameters(path_raw.flatten(-2))
        path_x = _scale_beta_mean(
            path_alpha[..., 0], path_beta[..., 0], PATH_X_MIN_M, PATH_X_MAX_M
        )
        path_y = _scale_beta_mean(
            path_alpha[..., 1], path_beta[..., 1], PATH_Y_MIN_M, PATH_Y_MAX_M
        )
        path_xy_m = torch.stack((path_x, path_y), dim=-1)
        speed_alpha, speed_beta = _beta_parameters(self.speed_beta_head(context))
        target_speed_mps = _scale_beta_mean(
            speed_alpha, speed_beta, 0.0, FIXED_BASE_SPEED_MPS
        )
        origin = torch.zeros(
            path_xy_m.shape[0], 1, 2,
            device=path_xy_m.device, dtype=path_xy_m.dtype,
        )
        action_logits = self.action_head(context)
        probabilities = torch.softmax(action_logits, dim=-1)
        return {
            "trajectory_xy_m": path_xy_m,
            "trajectory_with_origin_xy_m": torch.cat((origin, path_xy_m), 1),
            "target_speed_mps": target_speed_mps,
            "path_alpha": path_alpha,
            "path_beta": path_beta,
            "speed_alpha": speed_alpha,
            "speed_beta": speed_beta,
            "action_logits": action_logits,
            "action_probabilities": probabilities,
            "action_prediction": action_logits.argmax(dim=-1),
            "spatial_tokens": fused,
            "lateral_residual_m": path_xy_m[..., 1],
            "speed_delta_mps": target_speed_mps,
        }

    def load_compatible_state_dict(
        self, source_state: dict[str, torch.Tensor]
    ) -> dict[str, Any]:
        own = self.state_dict()
        excluded = (
            "route_encoder.", "route_fusion.", "action_head.",
            "lateral_head.", "speed_delta_head.", "path_beta_head.",
            "speed_beta_head.", "goal_encoder.", "regression_ego_adapter.",
        )
        transferred = {
            key: value for key, value in source_state.items()
            if key in own and own[key].shape == value.shape
            and not key.startswith(excluded)
            and key not in {"spatial_anchors_m", "temporal_anchors_s"}
        }
        missing, unexpected = self.load_state_dict(transferred, strict=False)
        return {
            "loaded_parameters": len(transferred),
            "fresh_parameters": sorted(missing),
            "unexpected_parameters": sorted(unexpected),
            "policy": "transfer sensor/temporal encoders only; reset route-conditioned fusion and all heads",
        }

    load_v8_state_dict = load_compatible_state_dict


__all__ = [
    "BETA_MAX_SHAPE", "BETA_MIN_SHAPE", "GoalTrajectoryPlannerV13",
    "ModelConfig", "PATH_X_MAX_M", "PATH_X_MIN_M", "PATH_Y_MAX_M",
    "PATH_Y_MIN_M",
]
