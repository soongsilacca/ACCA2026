from __future__ import annotations

from typing import Any

import numpy as np
import torch
import torch.nn as nn

from multimodal_planner_v5.model import ModelConfig
from multimodal_planner_v7.model import (
    MAX_LATERAL_RESIDUAL_M,
    MOTION_STATE_COUNT,
    XY_SCALE_M,
    RouteLockedSpeedPlannerV7,
    interpolate_route_by_progress,
)

# Runtime inference does not need the V8 training dataset (which inherits V6).
# Keep the exact training anchor contract locally to avoid importing the
# training-only dependency chain.
SPATIAL_ANCHOR_COUNT = 20
SPATIAL_ANCHORS_M = (
    np.arange(1, SPATIAL_ANCHOR_COUNT + 1, dtype=np.float32) * 3.0
)


class SpatialResidualPlannerV8(RouteLockedSpeedPlannerV7):
    """Predict only fixed-distance path-normal residual and STOP/DRIVE."""

    def __init__(self, config: ModelConfig | None = None) -> None:
        super().__init__(config)
        del self.speed_delta_head
        dim = self.config.hidden_dim
        # Recreate the head explicitly because V7 indices are temporal whereas
        # V8 indices are fixed spatial stations.
        self.lateral_head = nn.Sequential(
            nn.LayerNorm(dim),
            nn.Linear(dim, dim * 2),
            nn.GELU(),
            nn.Dropout(self.config.dropout),
            nn.Linear(dim * 2, SPATIAL_ANCHOR_COUNT),
        )
        self.register_buffer(
            "spatial_anchors_m",
            torch.as_tensor(SPATIAL_ANCHORS_M.copy()),
            persistent=True,
        )

    def forward(
        self,
        front: torch.Tensor,
        left: torch.Tensor,
        right: torch.Tensor,
        lidar_bev: torch.Tensor,
        ego: torch.Tensor,
        mgeo: torch.Tensor,
        local_route: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        context, fused = self._context(
            front,
            left,
            right,
            lidar_bev,
            ego,
            mgeo,
            local_route,
        )
        lateral_residual_m = (
            torch.tanh(self.lateral_head(context)) * MAX_LATERAL_RESIDUAL_M
        )
        stations = self.spatial_anchors_m.to(
            device=context.device,
            dtype=context.dtype,
        ).unsqueeze(0).expand(context.shape[0], -1)
        base_xy_m, _ = interpolate_route_by_progress(local_route, stations)
        # Derive the path normal and final yaw from geometry, never regress yaw.
        _, base_yaw = interpolate_route_by_progress(local_route, stations)
        normal = torch.stack((-torch.sin(base_yaw), torch.cos(base_yaw)), dim=-1)
        path_xy_m = base_xy_m + lateral_residual_m.unsqueeze(-1) * normal
        with_origin = torch.cat(
            (torch.zeros_like(path_xy_m[:, :1]), path_xy_m),
            dim=1,
        )
        step = with_origin[:, 1:] - with_origin[:, :-1]
        yaw = torch.atan2(step[..., 1], step[..., 0])
        state_logits = self.state_head(context)
        spatial_path = torch.cat(
            (
                path_xy_m / XY_SCALE_M,
                (yaw / torch.pi).unsqueeze(-1),
            ),
            dim=-1,
        )
        return {
            "lateral_residual_m": lateral_residual_m,
            "spatial_stations_m": stations,
            "spatial_path": spatial_path,
            "spatial_path_xy_m": path_xy_m,
            "spatial_path_yaw_rad": yaw,
            "motion_state_logits": state_logits,
            "motion_state_probabilities": torch.softmax(state_logits, dim=-1),
            "motion_state_prediction": state_logits.argmax(dim=-1),
            "spatial_tokens": fused,
        }

    def load_shared_state_dict(
        self,
        source_state: dict[str, torch.Tensor],
    ) -> dict[str, Any]:
        own = self.state_dict()
        excluded = ("trajectory_head.", "speed_delta_head.", "lateral_head.")
        transferable = {
            name: value
            for name, value in source_state.items()
            if name in own
            and own[name].shape == value.shape
            and not name.startswith(excluded)
            and name != "spatial_anchors_m"
        }
        incompatible = self.load_state_dict(transferable, strict=False)
        allowed_missing = {
            name
            for name in own
            if name.startswith("lateral_head.") or name == "spatial_anchors_m"
        }
        missing = set(incompatible.missing_keys)
        if missing != allowed_missing or incompatible.unexpected_keys:
            raise RuntimeError(
                "V8 transfer mismatch: "
                f"missing={sorted(missing)}, "
                f"unexpected={sorted(incompatible.unexpected_keys)}"
            )
        return {
            "loaded_parameters": len(transferable),
            "fresh_parameters": sorted(allowed_missing),
            "ignored_source_parameters": sorted(set(source_state) - set(transferable)),
        }

    def parameter_counts(self) -> dict[str, int]:
        counts = {
            "camera": sum(p.numel() for p in self.camera_encoder.parameters()),
            "lidar": sum(p.numel() for p in self.lidar_encoder.parameters()),
            "learned_speed_parameters": 0,
            "lateral_head": sum(p.numel() for p in self.lateral_head.parameters()),
            "state_head": sum(p.numel() for p in self.state_head.parameters()),
        }
        counts["total"] = sum(p.numel() for p in self.parameters())
        counts["trainable"] = sum(
            p.numel() for p in self.parameters() if p.requires_grad
        )
        return counts


__all__ = ["ModelConfig", "SpatialResidualPlannerV8"]
