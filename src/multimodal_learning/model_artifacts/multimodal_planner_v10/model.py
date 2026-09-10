from __future__ import annotations

from typing import Any

import torch
import torch.nn as nn

from multimodal_planner_v9.data import FIXED_BASE_SPEED_MPS
from multimodal_planner_v9.model import (
    MAX_LATERAL_RESIDUAL_M,
    XY_SCALE_M,
    ModelConfig,
    SpatialResidualSpeedPlannerV9,
    interpolate_route_by_progress,
    path_curvature_from_yaw,
)
from multimodal_planner_v10.data import TEMPORAL_ANCHORS_S


class TemporalResidualPlannerV10(SpatialResidualSpeedPlannerV9):
    """Six-point planner whose action branch receives no ego-state values."""

    architecture_name = "multimodal_planner_v10_temporal_residual"

    def __init__(self, config: ModelConfig | None = None) -> None:
        super().__init__(config)
        dim = self.config.hidden_dim
        dropout = self.config.dropout
        self.lateral_head = nn.Sequential(
            nn.LayerNorm(dim), nn.Linear(dim, dim * 2), nn.GELU(),
            nn.Dropout(dropout), nn.Linear(dim * 2, 6),
        )
        self.speed_delta_head = nn.Sequential(
            nn.LayerNorm(dim), nn.Linear(dim, dim * 2), nn.GELU(),
            nn.Dropout(dropout), nn.Linear(dim * 2, 6),
        )
        self.regression_ego_adapter = nn.Sequential(
            nn.LayerNorm(16), nn.Linear(16, dim), nn.GELU(),
            nn.Dropout(dropout), nn.Linear(dim, dim),
        )
        nn.init.zeros_(self.lateral_head[-1].weight)
        nn.init.zeros_(self.lateral_head[-1].bias)
        nn.init.zeros_(self.speed_delta_head[-1].weight)
        nn.init.constant_(self.speed_delta_head[-1].bias, -4.0)
        del self.speed_path_encoder
        del self.spatial_anchors_m
        self.speed_path_encoder = nn.Identity()
        self.register_buffer(
            "temporal_anchors_s",
            torch.as_tensor(TEMPORAL_ANCHORS_S.copy()),
            persistent=True,
        )
        # The shared trainer uses this legacy buffer name for resume checks.
        self.register_buffer(
            "spatial_anchors_m",
            torch.as_tensor(TEMPORAL_ANCHORS_S.copy()),
            persistent=True,
        )

    @staticmethod
    def _mask_classifier_ego(ego: torch.Tensor) -> torch.Tensor:
        # Keep the encoder/token topology checkpoint-compatible while removing
        # every ego/localization shortcut from the action context.
        return torch.zeros_like(ego)

    def forward(
        self,
        front: torch.Tensor,
        left: torch.Tensor,
        right: torch.Tensor,
        lidar_bev: torch.Tensor,
        ego: torch.Tensor,
        base_speed_profile_mps: torch.Tensor,
        local_route: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        # Action context receives camera, LiDAR and Local Route only. The zero
        # ego token is constant and therefore carries no sample information.
        context, fused = self._context(
            front, left, right, lidar_bev,
            self._mask_classifier_ego(ego), local_route,
        )
        action_logits = self.action_head(context)
        action_probabilities = torch.softmax(action_logits, dim=-1)

        # Regression keeps the complete ego vector, including speed.
        regression_context = context + self.regression_ego_adapter(ego[:, -1])
        lateral = torch.tanh(self.lateral_head(regression_context))
        lateral = lateral * MAX_LATERAL_RESIDUAL_M

        base_speed = torch.full_like(lateral, FIXED_BASE_SPEED_MPS)
        reduction = torch.sigmoid(self.speed_delta_head(regression_context))
        speed_delta = -reduction * base_speed
        candidate_speed = (base_speed + speed_delta).clamp_min(0.0)

        times = self.temporal_anchors_s.to(
            device=context.device, dtype=context.dtype
        ).unsqueeze(0).expand(context.shape[0], -1)
        # ego[...,0] is normalized by 30 m/s in the dataset.
        current_speed = ego[:, -1, 0].abs() * 30.0
        previous_time = torch.cat((torch.zeros_like(times[:, :1]), times[:, :-1]), 1)
        previous_speed = torch.cat((current_speed[:, None], candidate_speed[:, :-1]), 1)
        progress = torch.cumsum(
            0.5 * (previous_speed + candidate_speed) * (times - previous_time),
            dim=1,
        )
        base_xy, base_yaw = interpolate_route_by_progress(local_route, progress)
        normal = torch.stack((-torch.sin(base_yaw), torch.cos(base_yaw)), dim=-1)
        candidate_xy = base_xy + lateral.unsqueeze(-1) * normal
        with_origin = torch.cat((torch.zeros_like(candidate_xy[:, :1]), candidate_xy), 1)
        step = with_origin[:, 1:] - with_origin[:, :-1]
        candidate_yaw = torch.atan2(step[..., 1], step[..., 0])
        curvature = path_curvature_from_yaw(candidate_yaw, progress)

        base_with_origin = torch.cat((torch.zeros_like(base_xy[:, :1]), base_xy), 1)
        base_step = base_with_origin[:, 1:] - base_with_origin[:, :-1]
        base_path_yaw = torch.atan2(base_step[..., 1], base_step[..., 0])
        return {
            "lateral_residual_m": lateral,
            "speed_delta_mps": speed_delta,
            "base_speed_mps": base_speed,
            "candidate_spatial_speed_mps": candidate_speed,
            "temporal_lateral_residual_m": lateral,
            "temporal_speed_delta_mps": speed_delta,
            "temporal_candidate_speed_mps": candidate_speed,
            "temporal_anchors_s": times,
            "temporal_progress_m": progress,
            "temporal_candidate_path_xy_m": candidate_xy,
            # Compatibility aliases for V9 diagnostics.
            "spatial_stations_m": progress,
            "base_spatial_path_xy_m": base_xy,
            "base_spatial_path_yaw_rad": base_path_yaw,
            "candidate_spatial_path_xy_m": candidate_xy,
            "candidate_spatial_path_yaw_rad": candidate_yaw,
            "spatial_path_curvature_per_m": curvature,
            "base_spatial_path": torch.cat(
                (base_xy / XY_SCALE_M, (base_path_yaw / torch.pi).unsqueeze(-1)), -1
            ),
            "candidate_spatial_path": torch.cat(
                (candidate_xy / XY_SCALE_M, (candidate_yaw / torch.pi).unsqueeze(-1)), -1
            ),
            "speed_path_tokens": regression_context.unsqueeze(1).expand(-1, 6, -1),
            "action_logits": action_logits,
            "action_probabilities": action_probabilities,
            "action_prediction": action_logits.argmax(dim=-1),
            "spatial_tokens": fused,
        }

    def load_compatible_state_dict(
        self, source_state: dict[str, torch.Tensor]
    ) -> dict[str, Any]:
        own = self.state_dict()
        transferred = {
            key: value for key, value in source_state.items()
            if key in own and own[key].shape == value.shape
            and key not in {"spatial_anchors_m", "temporal_anchors_s"}
            and not key.startswith(
                ("action_head.", "lateral_head.", "speed_delta_head.")
            )
        }
        missing, unexpected = self.load_state_dict(transferred, strict=False)
        return {
            "loaded_parameters": len(transferred),
            "fresh_parameters": sorted(missing),
            "unexpected_parameters": sorted(unexpected),
        }

    load_v8_state_dict = load_compatible_state_dict


__all__ = ["ModelConfig", "TemporalResidualPlannerV10"]
