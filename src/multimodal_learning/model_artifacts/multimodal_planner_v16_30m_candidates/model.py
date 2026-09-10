from __future__ import annotations

from typing import Any

import torch
import torch.nn as nn
import torch.nn.functional as F

from multimodal_planner_v9.data import ACTION_NAMES, FIXED_BASE_SPEED_MPS
from multimodal_planner_v9.model import ModelConfig
from multimodal_planner_v10.model import TemporalResidualPlannerV10
from .data import (
    GOAL_LOOKAHEAD_M,
    PATH_ANCHORS_M,
    WAYPOINT_HORIZONS_S,
)


BETA_MIN_SHAPE = 1.01
BETA_MAX_SHAPE = 100.0
LATERAL_SCALE_M = 15.0


def _beta_parameters(raw: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    shapes = (F.softplus(raw.float()) + BETA_MIN_SHAPE).clamp_max(BETA_MAX_SHAPE)
    return shapes[..., 0], shapes[..., 1]


def _scale_beta_mean(
    alpha: torch.Tensor, beta: torch.Tensor, low: float, high: float
) -> torch.Tensor:
    unit = alpha / (alpha + beta).clamp_min(1.0e-6)
    return low + unit * (high - low)


def _head(dim: int, dropout: float, output_dim: int) -> nn.Sequential:
    return nn.Sequential(
        nn.LayerNorm(dim), nn.Linear(dim, dim * 2), nn.GELU(),
        nn.Dropout(dropout), nn.Linear(dim * 2, output_dim),
    )


class CumulativeWaypointDecoder(nn.Module):
    """TCP-style six-step decoder initialized to a 30 km/h straight path."""

    def __init__(self, hidden_dim: int) -> None:
        super().__init__()
        self.cell = nn.GRUCell(input_size=4, hidden_size=hidden_dim)
        self.output_delta = nn.Linear(hidden_dim, 2)
        nn.init.zeros_(self.output_delta.weight)
        nn.init.zeros_(self.output_delta.bias)
        anchor = torch.as_tensor(PATH_ANCHORS_M.copy(), dtype=torch.float32)
        origin = torch.zeros(1, dtype=torch.float32)
        forward_step = torch.diff(torch.cat((origin, anchor))) / GOAL_LOOKAHEAD_M
        self.register_buffer(
            "base_step_normalized",
            torch.stack((forward_step, torch.zeros_like(forward_step)), dim=-1),
        )
        # A bounded residual keeps the initial forward/backward pass finite but
        # still permits a large longitudinal change and multi-metre avoidance.
        self.register_buffer(
            "residual_scale_normalized",
            torch.tensor([0.25, 0.18], dtype=torch.float32),
        )
        self.register_buffer(
            "output_scale_m",
            torch.tensor([GOAL_LOOKAHEAD_M, LATERAL_SCALE_M], dtype=torch.float32),
        )

    def forward(
        self, context: torch.Tensor, goal_point: torch.Tensor
    ) -> torch.Tensor:
        hidden = context
        waypoint = torch.zeros(
            context.shape[0], 2, device=context.device, dtype=context.dtype
        )
        predictions: list[torch.Tensor] = []
        base_steps = self.base_step_normalized.to(context.dtype)
        residual_scale = self.residual_scale_normalized.to(context.dtype)
        output_scale = self.output_scale_m.to(context.dtype)
        for index in range(len(PATH_ANCHORS_M)):
            hidden = self.cell(torch.cat((waypoint, goal_point), dim=-1), hidden)
            residual = torch.tanh(self.output_delta(hidden)) * residual_scale
            waypoint = waypoint + base_steps[index] + residual
            predictions.append(waypoint * output_scale)
        return torch.stack(predictions, dim=1)


class GoalCandidatePlannerV16(TemporalResidualPlannerV10):
    """Camera3 + LiDAR + one 33.33 m goal; no route shape or ego input."""

    architecture_name = "multimodal_planner_v16_single_33m_goal_tcp6_candidates"

    def __init__(self, config: ModelConfig | None = None) -> None:
        super().__init__(config)
        dim = self.config.hidden_dim
        dropout = self.config.dropout
        for name in (
            "route_encoder", "regression_ego_adapter", "lateral_head",
            "speed_path_encoder", "speed_delta_head", "spatial_anchors_m",
            "temporal_anchors_s",
        ):
            if hasattr(self, name):
                delattr(self, name)
        self.goal_encoder = nn.Sequential(
            nn.LayerNorm(2), nn.Linear(2, dim), nn.GELU(),
            nn.Dropout(dropout), nn.Linear(dim, dim), nn.LayerNorm(dim),
        )
        self.drive_path_decoder = CumulativeWaypointDecoder(dim)
        self.avoid_path_decoder = CumulativeWaypointDecoder(dim)
        self.speed_beta_head = _head(dim, dropout, 2 * len(PATH_ANCHORS_M))
        nn.init.zeros_(self.speed_beta_head[-1].weight)
        nn.init.zeros_(self.speed_beta_head[-1].bias)
        self.register_buffer(
            "spatial_anchors_m",
            torch.as_tensor(PATH_ANCHORS_M.copy(), dtype=torch.float32),
        )
        self.register_buffer(
            "waypoint_horizons_s",
            torch.as_tensor(WAYPOINT_HORIZONS_S.copy(), dtype=torch.float32),
        )

    def _context(
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
        context, fused = self._context(front, left, right, lidar_bev, goal_point)
        drive_path = self.drive_path_decoder(context, goal_point)
        avoid_path = self.avoid_path_decoder(context, goal_point)
        speed_params = self.speed_beta_head(context).reshape(
            -1, len(PATH_ANCHORS_M), 2
        )
        speed_alpha, speed_beta = _beta_parameters(speed_params)
        target_speed = _scale_beta_mean(
            speed_alpha, speed_beta, 0.0, FIXED_BASE_SPEED_MPS
        )
        action_logits = self.action_head(context)
        action_probabilities = torch.softmax(action_logits, -1)
        atomic_output = torch.cat(
            (
                drive_path.flatten(1),
                avoid_path.flatten(1),
                target_speed,
                action_probabilities,
            ),
            dim=1,
        )
        origin = torch.zeros(
            drive_path.shape[0], 1, 2,
            device=drive_path.device, dtype=drive_path.dtype,
        )
        return {
            "drive_path_xy_m": drive_path,
            "avoid_path_xy_m": avoid_path,
            "drive_path_with_origin_xy_m": torch.cat((origin, drive_path), 1),
            "avoid_path_with_origin_xy_m": torch.cat((origin, avoid_path), 1),
            "target_speed_mps": target_speed,
            "speed_alpha": speed_alpha,
            "speed_beta": speed_beta,
            "action_logits": action_logits,
            "action_probabilities": action_probabilities,
            "action_prediction": action_logits.argmax(-1),
            "atomic_output": atomic_output,
            "spatial_tokens": fused,
            # Compatibility only: runtime selection remains outside the model.
            "trajectory_xy_m": drive_path,
            "lateral_residual_m": drive_path[..., 1],
            "speed_delta_mps": target_speed,
        }

    def load_compatible_state_dict(
        self, source_state: dict[str, torch.Tensor]
    ) -> dict[str, Any]:
        own = self.state_dict()
        fresh_prefixes = (
            "goal_encoder.", "drive_path_decoder.",
            "avoid_path_decoder.", "speed_beta_head.",
        )
        transferred = {
            key: value
            for key, value in source_state.items()
            if key in own
            and own[key].shape == value.shape
            and not key.startswith(fresh_prefixes)
            and key not in {
                "spatial_anchors_m", "temporal_anchors_s",
                "waypoint_horizons_s",
            }
        }
        missing, unexpected = self.load_state_dict(transferred, strict=False)
        return {
            "loaded_parameters": len(transferred),
            "fresh_parameters": sorted(missing),
            "unexpected_parameters": sorted(unexpected),
            "policy": (
                "transfer camera/LiDAR/query/temporal/fusion/action weights; "
                "initialize single-goal TCP-style DRIVE/AVOID decoders and speed head"
            ),
        }

    load_v8_state_dict = load_compatible_state_dict

    @staticmethod
    def _reset_module_parameters(module: nn.Module) -> None:
        for child in module.modules():
            reset = getattr(child, "reset_parameters", None)
            if callable(reset):
                reset()

    def reset_morai_deployment_heads(self, seed: int = 2026) -> list[str]:
        """Reset domain-specific heads after Bench2Drive pretraining.

        The sensor, temporal, fusion, goal and DRIVE/AVOID path decoder
        weights remain intact.  Only the MORAI-specific state classifier and
        absolute-speed distribution are made fresh.
        """
        with torch.random.fork_rng(devices=[]):
            torch.manual_seed(seed)
            self._reset_module_parameters(self.action_head)
            self._reset_module_parameters(self.speed_beta_head)
            nn.init.zeros_(self.speed_beta_head[-1].weight)
            nn.init.zeros_(self.speed_beta_head[-1].bias)
        return ["action_head", "speed_beta_head"]

    def parameter_counts(self) -> dict[str, int]:
        modules = {
            "camera": self.camera_encoder,
            "lidar": self.lidar_encoder,
            "constant_ego_encoder": self.ego_encoder,
            "goal": self.goal_encoder,
            "drive_path_decoder": self.drive_path_decoder,
            "avoid_path_decoder": self.avoid_path_decoder,
            "speed_head": self.speed_beta_head,
            "action_head": self.action_head,
        }
        result = {
            name: sum(parameter.numel() for parameter in module.parameters())
            for name, module in modules.items()
        }
        result["total"] = sum(parameter.numel() for parameter in self.parameters())
        result["trainable"] = sum(
            parameter.numel()
            for parameter in self.parameters()
            if parameter.requires_grad
        )
        return result

    def output_contract(self) -> dict[str, Any]:
        return {
            "inputs": [
                "front_camera_history", "left_camera_history",
                "right_camera_history", "lidar_bev_history",
                "single_goal_point_33.33m",
            ],
            "excluded_inputs": [
                "current_speed", "ego_state", "imu", "gps",
                "gps_health", "mgeo", "local_route_64x4",
                "intermediate_route_points", "route_command",
            ],
            "drive_path_xy_m": [len(PATH_ANCHORS_M), 2],
            "avoid_path_xy_m": [len(PATH_ANCHORS_M), 2],
            "target_speed_mps": [len(PATH_ANCHORS_M)],
            "waypoint_horizons_s": WAYPOINT_HORIZONS_S.tolist(),
            "nominal_path_anchors_m_at_30kph": PATH_ANCHORS_M.tolist(),
            "goal_lookahead_m": GOAL_LOOKAHEAD_M,
            "path_decoder": "TCP-style recurrent cumulative six-waypoint decoder",
            "action_state": list(ACTION_NAMES),
            "model_routing": "none; downstream state machine selects candidate",
            "atomic_output": {
                "shape": [33],
                "order": "drive_path12, avoid_path12, absolute_speed6, probabilities3",
            },
        }

    @staticmethod
    def label_policy() -> str:
        return (
            "explicit reviewed DRIVE/STOP/AVOID; single 33.33m goal only; "
            "DRIVE path uses Local Route offline as a label, never as model "
            "input; AVOID uses six existing raw future frames without "
            "interpolation or repair; STOP is classification-only"
        )


__all__ = [
    "CumulativeWaypointDecoder", "GoalCandidatePlannerV16",
    "LATERAL_SCALE_M", "ModelConfig",
]
