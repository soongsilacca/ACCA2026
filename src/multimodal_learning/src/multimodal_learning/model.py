from __future__ import annotations

from typing import Any

import torch
import torch.nn as nn
import torch.nn.functional as F

from multimodal_planner_v9.data import ACTION_NAMES, FIXED_BASE_SPEED_MPS
from multimodal_planner_v9.model import ModelConfig
from multimodal_planner_v10.model import TemporalResidualPlannerV10
from multimodal_planner_v16_30m_candidates.model import (
    BETA_MAX_SHAPE,
    BETA_MIN_SHAPE,
    LATERAL_SCALE_M,
    _head,
    _scale_beta_mean,
)
from .data import GOAL_LOOKAHEAD_M, SPATIAL_ANCHORS_M


def _beta_parameters(raw: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    shapes = (F.softplus(raw.float()) + BETA_MIN_SHAPE).clamp_max(BETA_MAX_SHAPE)
    return shapes[..., 0], shapes[..., 1]


class SpatialWaypointDecoder(nn.Module):
    """Six cumulative points initialized at fixed 30 m spatial stations."""

    def __init__(self, hidden_dim: int) -> None:
        super().__init__()
        self.cell = nn.GRUCell(input_size=4, hidden_size=hidden_dim)
        self.output_delta = nn.Linear(hidden_dim, 2)
        nn.init.zeros_(self.output_delta.weight)
        nn.init.zeros_(self.output_delta.bias)
        anchor = torch.as_tensor(SPATIAL_ANCHORS_M.copy(), dtype=torch.float32)
        forward_step = torch.diff(torch.cat((torch.zeros(1), anchor))) / GOAL_LOOKAHEAD_M
        self.register_buffer(
            "base_step_normalized",
            torch.stack((forward_step, torch.zeros_like(forward_step)), dim=-1),
        )
        self.register_buffer(
            "residual_scale_normalized", torch.tensor([0.20, 0.18], dtype=torch.float32)
        )
        self.register_buffer(
            "output_scale_m", torch.tensor([GOAL_LOOKAHEAD_M, LATERAL_SCALE_M], dtype=torch.float32)
        )

    def forward(self, context: torch.Tensor, goal_point: torch.Tensor) -> torch.Tensor:
        hidden = context
        waypoint = torch.zeros(context.shape[0], 2, device=context.device, dtype=context.dtype)
        predictions: list[torch.Tensor] = []
        for index in range(len(SPATIAL_ANCHORS_M)):
            hidden = self.cell(torch.cat((waypoint, goal_point), -1), hidden)
            residual = torch.tanh(self.output_delta(hidden)) * self.residual_scale_normalized.to(context.dtype)
            waypoint = waypoint + self.base_step_normalized[index].to(context.dtype) + residual
            predictions.append(waypoint * self.output_scale_m.to(context.dtype))
        return torch.stack(predictions, 1)


class GoalSpatialCandidatePlannerV17(TemporalResidualPlannerV10):
    architecture_name = "multimodal_planner_v17_fixed_spatial_30m_candidates"

    def __init__(self, config: ModelConfig | None = None) -> None:
        super().__init__(config)
        dim, dropout = self.config.hidden_dim, self.config.dropout
        for name in (
            "route_encoder", "regression_ego_adapter", "lateral_head",
            "speed_path_encoder", "speed_delta_head", "spatial_anchors_m",
            "temporal_anchors_s",
        ):
            if hasattr(self, name):
                delattr(self, name)
        self.goal_encoder = nn.Sequential(
            nn.LayerNorm(2), nn.Linear(2, dim), nn.GELU(), nn.Dropout(dropout),
            nn.Linear(dim, dim), nn.LayerNorm(dim),
        )
        self.drive_path_decoder = SpatialWaypointDecoder(dim)
        self.avoid_path_decoder = SpatialWaypointDecoder(dim)
        self.speed_beta_head = _head(dim, dropout, 2 * len(SPATIAL_ANCHORS_M))
        nn.init.zeros_(self.speed_beta_head[-1].weight)
        nn.init.zeros_(self.speed_beta_head[-1].bias)
        self.register_buffer("spatial_anchors_m", torch.as_tensor(SPATIAL_ANCHORS_M.copy()))

    def _context(self, front, left, right, lidar_bev, goal_point):
        if goal_point.ndim != 2 or goal_point.shape[1] != 2:
            raise ValueError("goal_point must have shape [batch,2]")
        batch, history = front.shape[:2]
        no_ego = torch.zeros(batch, history, 16, device=front.device, dtype=front.dtype)
        frame_slots = self.encode_frames(front, left, right, lidar_bev, no_ego)
        _, _, slots, dim = frame_slots.shape
        temporal_input = frame_slots.permute(0, 2, 1, 3).reshape(batch * slots, history, dim)
        _, hidden = self.temporal_gru(temporal_input)
        fused = hidden[-1].reshape(batch, slots, dim)
        goal_token = self.goal_encoder(goal_point).unsqueeze(1)
        for layer in self.route_fusion:
            fused = layer(fused, goal_token)
        fused = self.output_norm(fused)
        return fused.mean(1), fused

    def forward(self, front, left, right, lidar_bev, goal_point):
        context, fused = self._context(front, left, right, lidar_bev, goal_point)
        drive_path = self.drive_path_decoder(context, goal_point)
        avoid_path = self.avoid_path_decoder(context, goal_point)
        speed_params = self.speed_beta_head(context).reshape(-1, 6, 2)
        speed_alpha, speed_beta = _beta_parameters(speed_params)
        target_speed = _scale_beta_mean(speed_alpha, speed_beta, 0.0, FIXED_BASE_SPEED_MPS)
        action_logits = self.action_head(context)
        probabilities = torch.softmax(action_logits, -1)
        atomic = torch.cat((drive_path.flatten(1), avoid_path.flatten(1), target_speed, probabilities), 1)
        origin = torch.zeros(drive_path.shape[0], 1, 2, device=drive_path.device, dtype=drive_path.dtype)
        return {
            "drive_path_xy_m": drive_path,
            "avoid_path_xy_m": avoid_path,
            "drive_path_with_origin_xy_m": torch.cat((origin, drive_path), 1),
            "avoid_path_with_origin_xy_m": torch.cat((origin, avoid_path), 1),
            "target_speed_mps": target_speed,
            "speed_alpha": speed_alpha,
            "speed_beta": speed_beta,
            "action_logits": action_logits,
            "action_probabilities": probabilities,
            "action_prediction": action_logits.argmax(-1),
            "atomic_output": atomic,
            "spatial_tokens": fused,
            "trajectory_xy_m": drive_path,
            "lateral_residual_m": drive_path[..., 1],
            "speed_delta_mps": target_speed,
        }

    def load_compatible_state_dict(self, source_state: dict[str, torch.Tensor]) -> dict[str, Any]:
        own = self.state_dict()
        protected_buffers = {
            "spatial_anchors_m",
            "drive_path_decoder.base_step_normalized",
            "drive_path_decoder.output_scale_m",
            "avoid_path_decoder.base_step_normalized",
            "avoid_path_decoder.output_scale_m",
        }
        transferred = {
            key: value for key, value in source_state.items()
            if key in own and own[key].shape == value.shape and key not in protected_buffers
        }
        missing, unexpected = self.load_state_dict(transferred, strict=False)
        return {
            "loaded_parameters": len(transferred),
            "fresh_parameters": sorted(missing),
            "unexpected_parameters": sorted(unexpected),
            "policy": "transfer compatible sensor/fusion/head weights; preserve V17 fixed-spatial buffers",
        }

    load_v8_state_dict = load_compatible_state_dict

    @staticmethod
    def _reset_module_parameters(module: nn.Module) -> None:
        for child in module.modules():
            reset = getattr(child, "reset_parameters", None)
            if callable(reset):
                reset()

    def reset_morai_deployment_heads(self, seed: int = 2026) -> list[str]:
        with torch.random.fork_rng(devices=[]):
            torch.manual_seed(seed)
            self._reset_module_parameters(self.action_head)
            self._reset_module_parameters(self.speed_beta_head)
            nn.init.zeros_(self.speed_beta_head[-1].weight)
            nn.init.zeros_(self.speed_beta_head[-1].bias)
        return ["action_head", "speed_beta_head"]

    def parameter_counts(self) -> dict[str, int]:
        modules = {
            "camera": self.camera_encoder, "lidar": self.lidar_encoder,
            "goal": self.goal_encoder, "drive_path_decoder": self.drive_path_decoder,
            "avoid_path_decoder": self.avoid_path_decoder,
            "speed_head": self.speed_beta_head, "action_head": self.action_head,
        }
        result = {name: sum(p.numel() for p in module.parameters()) for name, module in modules.items()}
        result["total"] = sum(p.numel() for p in self.parameters())
        result["trainable"] = sum(p.numel() for p in self.parameters() if p.requires_grad)
        return result

    def output_contract(self) -> dict[str, Any]:
        return {
            "inputs": ["camera3_history", "lidar_bev_history", "single_goal_point_30m"],
            "excluded_inputs": ["current_speed", "ego", "gps", "mgeo", "local_route"],
            "drive_path_xy_m": [6, 2], "avoid_path_xy_m": [6, 2],
            "target_speed_mps": [6], "spatial_anchors_m": SPATIAL_ANCHORS_M.tolist(),
            "anchor_axis": "route_progress_m", "temporal_path_target": False,
            "action_state": list(ACTION_NAMES), "atomic_output": [33],
        }

    @staticmethod
    def label_policy() -> str:
        return "fixed spatial 3/6/10/15/22/30m; route is offline label only; no current-speed requirement for path"


__all__ = ["GoalSpatialCandidatePlannerV17", "ModelConfig", "SpatialWaypointDecoder"]
