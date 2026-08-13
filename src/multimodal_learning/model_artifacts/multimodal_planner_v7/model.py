from __future__ import annotations

from typing import Any

import torch
import torch.nn as nn

from multimodal_planner_v5.model import (
    ModelConfig,
    MultiViewTemporalTrajectoryPlannerV5,
    prepend_local_origin,
)


HORIZON_DT_SEC = 0.2
XY_SCALE_M = 50.0
YAW_SCALE_RAD = torch.pi
SPEED_SCALE_MPS = 20.0
MAX_SPEED_MPS = 55.0
MAX_SPEED_DELTA_MPS = 55.0
MAX_LATERAL_RESIDUAL_M = 5.0
MOTION_STATE_COUNT = 2


def _route_geometry(
    local_route: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """Return route points, segments, lengths and origin projection progress.

    The dataset stores route XY normalized by 50 m. The closest projection of
    ego origin (0, 0) onto the polyline is used as s=0, so route points behind
    the vehicle do not consume predicted forward progress.
    """
    if local_route.ndim != 3 or local_route.shape[-2:] != (64, 4):
        raise ValueError(
            "local_route must have shape [batch,64,4], "
            f"got {tuple(local_route.shape)}"
        )
    points_m = local_route[..., :2] * XY_SCALE_M
    segments = points_m[:, 1:] - points_m[:, :-1]
    lengths = torch.linalg.vector_norm(segments, dim=-1).clamp_min(1.0e-4)
    cumulative = torch.cat(
        (
            torch.zeros_like(lengths[:, :1]),
            torch.cumsum(lengths, dim=1),
        ),
        dim=1,
    )

    start = points_m[:, :-1]
    denom = lengths.square()
    projection_fraction = (
        -(start * segments).sum(dim=-1) / denom
    ).clamp(0.0, 1.0)
    projected = start + projection_fraction.unsqueeze(-1) * segments
    distance_sq = projected.square().sum(dim=-1)
    closest_segment = distance_sq.argmin(dim=1)
    batch_index = torch.arange(points_m.shape[0], device=points_m.device)
    origin_s = (
        cumulative[batch_index, closest_segment]
        + projection_fraction[batch_index, closest_segment]
        * lengths[batch_index, closest_segment]
    )
    return points_m, segments, lengths, origin_s


def interpolate_route_by_progress(
    local_route: torch.Tensor,
    forward_progress_m: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Interpolate route XY and tangent yaw at non-negative ego-relative s.

    Progress beyond the last route point is linearly extrapolated using the
    final route segment. This avoids freezing fast trajectories at the 64 m
    route boundary.
    """
    if forward_progress_m.ndim != 2:
        raise ValueError(
            "forward_progress_m must have shape [batch,horizon], "
            f"got {tuple(forward_progress_m.shape)}"
        )
    points_m, segments, lengths, origin_s = _route_geometry(local_route)
    cumulative = torch.cat(
        (
            torch.zeros_like(lengths[:, :1]),
            torch.cumsum(lengths, dim=1),
        ),
        dim=1,
    )
    absolute_s = origin_s.unsqueeze(1) + forward_progress_m.clamp_min(0.0)
    route_end = cumulative[:, -1:]
    search_s = torch.minimum(absolute_s, route_end)
    segment_index = torch.searchsorted(
        cumulative.contiguous(),
        search_s.contiguous(),
        right=True,
    ) - 1
    segment_index = segment_index.clamp(0, segments.shape[1] - 1)

    gather_xy = segment_index.unsqueeze(-1).expand(-1, -1, 2)
    segment_start = torch.gather(points_m[:, :-1], 1, gather_xy)
    segment = torch.gather(segments, 1, gather_xy)
    segment_length = torch.gather(lengths, 1, segment_index)
    segment_s = torch.gather(cumulative[:, :-1], 1, segment_index)
    fraction = ((search_s - segment_s) / segment_length).clamp(0.0, 1.0)
    xy_m = segment_start + fraction.unsqueeze(-1) * segment

    overflow = (absolute_s - route_end).clamp_min(0.0)
    last_unit = segments[:, -1] / lengths[:, -1:].clamp_min(1.0e-4)
    xy_m = xy_m + overflow.unsqueeze(-1) * last_unit.unsqueeze(1)
    yaw_rad = torch.atan2(segment[..., 1], segment[..., 0])
    return xy_m, yaw_rad


def project_points_to_route(
    local_route: torch.Tensor,
    points_m: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Project points onto the route and return ego-relative s and signed d."""
    if points_m.ndim != 3 or points_m.shape[-1] != 2:
        raise ValueError(
            "points_m must have shape [batch,points,2], "
            f"got {tuple(points_m.shape)}"
        )
    route_points, segments, lengths, origin_s = _route_geometry(local_route)
    cumulative = torch.cat(
        (
            torch.zeros_like(lengths[:, :1]),
            torch.cumsum(lengths, dim=1),
        ),
        dim=1,
    )
    segment_start = route_points[:, None, :-1, :]
    segment = segments[:, None, :, :]
    point_delta = points_m[:, :, None, :] - segment_start
    fraction = (
        (point_delta * segment).sum(dim=-1)
        / lengths[:, None, :].square()
    ).clamp(0.0, 1.0)
    projected = segment_start + fraction.unsqueeze(-1) * segment
    residual = points_m[:, :, None, :] - projected
    distance_sq = residual.square().sum(dim=-1)
    closest = distance_sq.argmin(dim=-1)
    gather_xy = closest.unsqueeze(-1).unsqueeze(-1).expand(-1, -1, 1, 2)
    closest_residual = torch.gather(residual, 2, gather_xy).squeeze(2)
    closest_segment = torch.gather(
        segments[:, None].expand(-1, points_m.shape[1], -1, -1),
        2,
        gather_xy,
    ).squeeze(2)
    closest_length = torch.gather(
        lengths[:, None].expand(-1, points_m.shape[1], -1),
        2,
        closest.unsqueeze(-1),
    ).squeeze(-1)
    closest_fraction = torch.gather(
        fraction,
        2,
        closest.unsqueeze(-1),
    ).squeeze(-1)
    segment_s = torch.gather(
        cumulative[:, None, :-1].expand(-1, points_m.shape[1], -1),
        2,
        closest.unsqueeze(-1),
    ).squeeze(-1)
    progress_m = (
        segment_s + closest_fraction * closest_length - origin_s.unsqueeze(1)
    )
    unit_tangent = closest_segment / closest_length.unsqueeze(-1).clamp_min(
        1.0e-4
    )
    signed_lateral_m = (
        unit_tangent[..., 0] * closest_residual[..., 1]
        - unit_tangent[..., 1] * closest_residual[..., 0]
    )
    return progress_m, signed_lateral_m


def speed_profile_to_trajectory(
    future_speed_normalized: torch.Tensor,
    local_route: torch.Tensor,
    lateral_residual_m: torch.Tensor | None = None,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Convert future speeds and path-normal residuals into a trajectory."""
    if future_speed_normalized.ndim != 2:
        raise ValueError(
            "future_speed_normalized must have shape [batch,horizon], "
            f"got {tuple(future_speed_normalized.shape)}"
        )
    speed_mps = future_speed_normalized * SPEED_SCALE_MPS
    progress_m = torch.cumsum(speed_mps * HORIZON_DT_SEC, dim=1)
    base_xy_m, base_yaw_rad = interpolate_route_by_progress(
        local_route,
        progress_m,
    )
    if lateral_residual_m is None:
        lateral_residual_m = torch.zeros_like(future_speed_normalized)
    if lateral_residual_m.shape != future_speed_normalized.shape:
        raise ValueError(
            "lateral_residual_m must match future speed shape, "
            f"got {tuple(lateral_residual_m.shape)}"
        )
    path_normal = torch.stack(
        (-torch.sin(base_yaw_rad), torch.cos(base_yaw_rad)),
        dim=-1,
    )
    xy_m = base_xy_m + lateral_residual_m.unsqueeze(-1) * path_normal
    xy_with_origin = torch.cat((torch.zeros_like(xy_m[:, :1]), xy_m), dim=1)
    path_step = xy_with_origin[:, 1:] - xy_with_origin[:, :-1]
    path_step_norm = torch.linalg.vector_norm(path_step, dim=-1)
    derived_yaw = torch.atan2(path_step[..., 1], path_step[..., 0])
    yaw_rad = torch.where(
        path_step_norm > 1.0e-3,
        derived_yaw,
        base_yaw_rad,
    )
    trajectory = torch.cat(
        (
            xy_m / XY_SCALE_M,
            (yaw_rad / YAW_SCALE_RAD).unsqueeze(-1),
            future_speed_normalized.unsqueeze(-1),
        ),
        dim=-1,
    )
    return trajectory, progress_m


class RouteLockedSpeedPlannerV7(MultiViewTemporalTrajectoryPlannerV5):
    """Keep the V5 encoders but predict speed along the supplied Local Route."""

    def __init__(
        self,
        config: ModelConfig | None = None,
        normal_base_speed_mps: float = 59.0 / 3.6,
        speed_zone_base_speed_mps: float = 20.0,
    ) -> None:
        super().__init__(config)
        del self.trajectory_head
        dim = self.config.hidden_dim
        if normal_base_speed_mps <= 0.0 or speed_zone_base_speed_mps <= 0.0:
            raise ValueError("fixed MGeo base speeds must be positive")
        self.normal_base_speed_mps = float(normal_base_speed_mps)
        self.speed_zone_base_speed_mps = float(speed_zone_base_speed_mps)
        self.speed_delta_head = nn.Sequential(
            nn.LayerNorm(dim),
            nn.Linear(dim, dim * 2),
            nn.GELU(),
            nn.Dropout(self.config.dropout),
            nn.Linear(dim * 2, self.config.horizon),
        )
        self.lateral_head = nn.Sequential(
            nn.LayerNorm(dim),
            nn.Linear(dim, dim * 2),
            nn.GELU(),
            nn.Dropout(self.config.dropout),
            nn.Linear(dim * 2, self.config.horizon),
        )
        self.state_head = nn.Sequential(
            nn.LayerNorm(dim),
            nn.Linear(dim, dim),
            nn.GELU(),
            nn.Dropout(self.config.dropout),
            nn.Linear(dim, MOTION_STATE_COUNT),
        )

    def _context(
        self,
        front: torch.Tensor,
        left: torch.Tensor,
        right: torch.Tensor,
        lidar_bev: torch.Tensor,
        ego: torch.Tensor,
        mgeo: torch.Tensor,
        local_route: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        frame_slots = self.encode_frames(front, left, right, lidar_bev, ego)
        batch, history, slots, dim = frame_slots.shape
        temporal_input = frame_slots.permute(0, 2, 1, 3).reshape(
            batch * slots,
            history,
            dim,
        )
        _, hidden = self.temporal_gru(temporal_input)
        fused = hidden[-1].reshape(batch, slots, dim)
        mgeo_tokens = self.mgeo_encoder(mgeo) + self.map_type_embedding[0]
        route_tokens = self.route_encoder(local_route) + self.map_type_embedding[1]
        map_route_tokens = torch.cat((mgeo_tokens, route_tokens), dim=1)
        for layer in self.route_fusion:
            fused = layer(fused, map_route_tokens)
        fused = self.output_norm(fused)
        return fused.mean(dim=1), fused

    def fixed_mgeo_base_speed(self, mgeo: torch.Tensor) -> torch.Tensor:
        """Map the non-learned MGeo speed-zone flag to a fixed speed profile."""
        if mgeo.ndim != 3 or mgeo.shape[-2:] != (64, 8):
            raise ValueError(
                "mgeo must have shape [batch,64,8], "
                f"got {tuple(mgeo.shape)}"
            )
        # The current schema stores only a binary speed-zone flag at index 7.
        # Majority over the near-forward tokens avoids one isolated boundary
        # token flipping the entire four-second speed prior.
        speed_zone = (mgeo[:, :16, 7].mean(dim=1) >= 0.5).to(mgeo.dtype)
        base_mps = (
            self.normal_base_speed_mps
            + speed_zone
            * (
                self.speed_zone_base_speed_mps
                - self.normal_base_speed_mps
            )
        )
        return base_mps.unsqueeze(1).expand(-1, self.config.horizon)

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
        base_speed_mps = self.fixed_mgeo_base_speed(mgeo)
        base_speed = base_speed_mps / SPEED_SCALE_MPS
        speed_delta = torch.tanh(self.speed_delta_head(context)) * (
            MAX_SPEED_DELTA_MPS / SPEED_SCALE_MPS
        )
        future_speed = torch.clamp(
            base_speed + speed_delta,
            min=0.0,
            max=MAX_SPEED_MPS / SPEED_SCALE_MPS,
        )
        lateral_residual_m = (
            torch.tanh(self.lateral_head(context)) * MAX_LATERAL_RESIDUAL_M
        )
        trajectory, progress_m = speed_profile_to_trajectory(
            future_speed,
            local_route,
            lateral_residual_m,
        )
        state_logits = self.state_head(context)
        return {
            "trajectory": trajectory,
            "trajectory_with_origin": prepend_local_origin(trajectory),
            "future_speed": future_speed,
            "base_speed": base_speed,
            "speed_delta": speed_delta,
            "lateral_residual_m": lateral_residual_m,
            "forward_progress_m": progress_m,
            "motion_state_logits": state_logits,
            "motion_state_probabilities": torch.softmax(state_logits, dim=-1),
            "motion_state_prediction": state_logits.argmax(dim=-1),
            "spatial_tokens": fused,
        }

    def load_encoder_state_dict(
        self,
        source_state: dict[str, torch.Tensor],
    ) -> dict[str, Any]:
        own = self.state_dict()
        transferable = {
            name: value
            for name, value in source_state.items()
            if name in own
            and own[name].shape == value.shape
            and not name.startswith(("trajectory_head.", "state_head."))
        }
        incompatible = self.load_state_dict(transferable, strict=False)
        allowed_missing = {
            name
            for name in own
            if name.startswith(("speed_delta_head.", "state_head."))
            or name.startswith("lateral_head.")
        }
        missing = set(incompatible.missing_keys)
        if missing != allowed_missing or incompatible.unexpected_keys:
            raise RuntimeError(
                "V7 encoder transfer mismatch: "
                f"missing={sorted(missing)}, "
                f"unexpected={sorted(incompatible.unexpected_keys)}"
            )
        return {
            "loaded_encoder_parameters": len(transferable),
            "new_head_parameters": sorted(allowed_missing),
            "ignored_source_parameters": sorted(
                set(source_state) - set(transferable)
            ),
        }

    def parameter_counts(self) -> dict[str, int]:
        counts = {
            "camera": sum(p.numel() for p in self.camera_encoder.parameters()),
            "lidar": sum(p.numel() for p in self.lidar_encoder.parameters()),
            "base_speed_parameters": 0,
            "speed_delta_head": sum(
                p.numel() for p in self.speed_delta_head.parameters()
            ),
            "lateral_head": sum(
                p.numel() for p in self.lateral_head.parameters()
            ),
            "state_head": sum(p.numel() for p in self.state_head.parameters()),
        }
        counts["total"] = sum(p.numel() for p in self.parameters())
        counts["trainable"] = sum(
            p.numel() for p in self.parameters() if p.requires_grad
        )
        return counts


__all__ = [
    "HORIZON_DT_SEC",
    "MAX_SPEED_MPS",
    "MAX_SPEED_DELTA_MPS",
    "MAX_LATERAL_RESIDUAL_M",
    "MOTION_STATE_COUNT",
    "ModelConfig",
    "RouteLockedSpeedPlannerV7",
    "SPEED_SCALE_MPS",
    "XY_SCALE_M",
    "interpolate_route_by_progress",
    "project_points_to_route",
    "speed_profile_to_trajectory",
]
