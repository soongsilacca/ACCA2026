from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class VelocityPlannerConfig:
    competition_max_speed_mps: float = 25.0
    max_lateral_acceleration_mps2: float = 3.0
    max_acceleration_mps2: float = 2.5
    max_deceleration_mps2: float = 5.0
    path_length_m: float = 80.0
    path_interval_m: float = 0.1
    stop_margin_m: float = 2.0


def _route_from_origin(
    route_xy_m: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float]:
    route = np.asarray(route_xy_m, dtype=np.float64)
    if route.ndim != 2 or route.shape[1] != 2 or len(route) < 2:
        raise ValueError("route_xy_m must have shape [points,2]")
    segments = np.diff(route, axis=0)
    lengths = np.maximum(np.linalg.norm(segments, axis=1), 1.0e-6)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    fraction = np.clip(
        np.sum((-route[:-1]) * segments, axis=1) / np.square(lengths),
        0.0,
        1.0,
    )
    projected = route[:-1] + fraction[:, None] * segments
    closest = int(np.argmin(np.sum(np.square(projected), axis=1)))
    origin_s = cumulative[closest] + fraction[closest] * lengths[closest]
    return route, cumulative, float(origin_s)


def _interpolate_route(
    route: np.ndarray,
    cumulative: np.ndarray,
    absolute_s: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    if float(absolute_s[-1]) > float(cumulative[-1]) + 1.0e-6:
        raise ValueError(
            f"route has only {cumulative[-1]:.2f}m total arc length but "
            f"{absolute_s[-1]:.2f}m is required; straight extrapolation is disabled"
        )
    x = np.interp(absolute_s, cumulative, route[:, 0])
    y = np.interp(absolute_s, cumulative, route[:, 1])
    xy = np.stack((x, y), axis=1)
    derivative = np.gradient(xy, absolute_s, axis=0, edge_order=1)
    norm = np.maximum(np.linalg.norm(derivative, axis=1), 1.0e-6)
    tangent = derivative / norm[:, None]
    return xy, tangent


def build_mpc_path(
    route_xy_m: np.ndarray,
    lateral_residual_m: np.ndarray,
    residual_anchors_m: np.ndarray,
    *,
    path_length_m: float = 80.0,
    interval_m: float = 0.1,
    route_start_index: int | None = None,
    lateral_deadband_m: float = 0.1,
) -> dict[str, np.ndarray]:
    """Apply learned near-field Δd to an independently supplied long route."""
    route, cumulative, origin_s = _route_from_origin(route_xy_m)
    if route_start_index is not None:
        start_index = int(route_start_index)
        if start_index < 0 or start_index >= len(route):
            raise ValueError(
                f"route_start_index {start_index} is outside route with "
                f"{len(route)} points"
            )
        origin_s = float(cumulative[start_index])
    if cumulative[-1] - origin_s < path_length_m:
        raise ValueError(
            f"only {cumulative[-1] - origin_s:.2f}m of route remains ahead; "
            f"{path_length_m:.2f}m is required"
        )
    anchors = np.asarray(residual_anchors_m, dtype=np.float64).reshape(-1)
    residual = np.asarray(lateral_residual_m, dtype=np.float64).reshape(-1)
    if anchors.shape != residual.shape or np.any(np.diff(anchors) <= 0.0):
        raise ValueError("residual anchors and values must match and increase")
    if lateral_deadband_m < 0.0:
        raise ValueError("lateral_deadband_m cannot be negative")
    residual = np.where(
        np.abs(residual) <= lateral_deadband_m,
        0.0,
        residual,
    )
    station = np.arange(
        0.0,
        path_length_m + interval_m * 0.5,
        interval_m,
        dtype=np.float64,
    )
    base_xy, tangent = _interpolate_route(
        route,
        cumulative,
        origin_s + station,
    )
    d_value = np.interp(
        np.minimum(station, anchors[-1]),
        anchors,
        residual,
    )
    tail = station > anchors[-1]
    if tail.any():
        tail_span = max(path_length_m - anchors[-1], interval_m)
        u = np.clip((station[tail] - anchors[-1]) / tail_span, 0.0, 1.0)
        smooth = 6.0 * u**5 - 15.0 * u**4 + 10.0 * u**3
        d_value[tail] = residual[-1] * (1.0 - smooth)
    normal = np.stack((-tangent[:, 1], tangent[:, 0]), axis=1)
    path_xy = base_xy + d_value[:, None] * normal
    derivative = np.gradient(path_xy, station, axis=0, edge_order=1)
    yaw = np.unwrap(np.arctan2(derivative[:, 1], derivative[:, 0]))
    curvature = np.gradient(yaw, station, edge_order=1)
    return {
        "station_m": station.astype(np.float32),
        "xy_m": path_xy.astype(np.float32),
        "yaw_rad": yaw.astype(np.float32),
        "curvature_per_m": curvature.astype(np.float32),
        "lateral_residual_m": d_value.astype(np.float32),
    }


def plan_velocity_profile(
    path: dict[str, np.ndarray],
    current_speed_mps: float,
    config: VelocityPlannerConfig = VelocityPlannerConfig(),
    *,
    stop_distance_m: float | None = None,
    obstacle_distance_m: float | None = None,
    drive: bool = True,
) -> np.ndarray:
    """Produce a maximum-feasible reference speed; no learned speed is used."""
    station = np.asarray(path["station_m"], dtype=np.float64)
    curvature = np.abs(np.asarray(path["curvature_per_m"], dtype=np.float64))
    curve_limit = np.sqrt(
        config.max_lateral_acceleration_mps2 / np.maximum(curvature, 1.0e-5)
    )
    speed = np.minimum(curve_limit, config.competition_max_speed_mps)
    constraints = [
        value
        for value in (stop_distance_m, obstacle_distance_m)
        if value is not None
    ]
    if not drive:
        constraints.append(0.0)
    if constraints:
        stop_at = max(min(constraints) - config.stop_margin_m, 0.0)
        remaining = np.maximum(stop_at - station, 0.0)
        braking_limit = np.sqrt(2.0 * config.max_deceleration_mps2 * remaining)
        speed = np.minimum(speed, braking_limit)
        speed[station >= stop_at] = 0.0

    ds = np.diff(station)
    for index in range(len(speed) - 2, -1, -1):
        reachable = np.sqrt(
            speed[index + 1] ** 2
            + 2.0 * config.max_deceleration_mps2 * ds[index]
        )
        speed[index] = min(speed[index], reachable)
    speed[0] = min(max(float(current_speed_mps), 0.0), speed[0])
    for index in range(1, len(speed)):
        reachable = np.sqrt(
            speed[index - 1] ** 2
            + 2.0 * config.max_acceleration_mps2 * ds[index - 1]
        )
        speed[index] = min(speed[index], reachable)
    return speed.astype(np.float32)


def plan_curvature_speed_profile(
    path: dict[str, np.ndarray],
    speed_limit_mps: float,
    *,
    max_lateral_acceleration_mps2: float = 3.0,
    max_deceleration_mps2: float = 2.0,
    curvature_smoothing_m: float = 1.0,
) -> np.ndarray:
    """Apply only curvature and preceding-braking limits to a road speed."""
    if not np.isfinite(speed_limit_mps) or speed_limit_mps < 0.0:
        raise ValueError("speed_limit_mps must be finite and non-negative")
    if max_lateral_acceleration_mps2 <= 0.0:
        raise ValueError("max_lateral_acceleration_mps2 must be positive")
    if max_deceleration_mps2 <= 0.0:
        raise ValueError("max_deceleration_mps2 must be positive")
    if curvature_smoothing_m < 0.0:
        raise ValueError("curvature_smoothing_m cannot be negative")

    station = np.asarray(path["station_m"], dtype=np.float64)
    curvature = np.abs(
        np.asarray(path["curvature_per_m"], dtype=np.float64)
    )
    if station.ndim != 1 or curvature.shape != station.shape or len(station) < 2:
        raise ValueError("path station and curvature must be matching vectors")
    ds = np.diff(station)
    if np.any(ds <= 0.0):
        raise ValueError("path station must increase")

    if curvature_smoothing_m > 0.0:
        sigma_samples = curvature_smoothing_m / float(np.median(ds))
        radius = max(int(math.ceil(3.0 * sigma_samples)), 1)
        offset = np.arange(-radius, radius + 1, dtype=np.float64)
        kernel = np.exp(-0.5 * np.square(offset / sigma_samples))
        kernel /= np.sum(kernel)
        padded = np.pad(curvature, radius, mode="edge")
        curvature = np.convolve(padded, kernel, mode="valid")

    curve_limit = np.sqrt(
        max_lateral_acceleration_mps2 / np.maximum(curvature, 1.0e-5)
    )
    speed = np.minimum(curve_limit, float(speed_limit_mps))

    # Propagate each curve limit backwards so deceleration begins before the
    # curve instead of stepping the scalar MPC target down at curve entry.
    for index in range(len(speed) - 2, -1, -1):
        reachable = np.sqrt(
            speed[index + 1] ** 2
            + 2.0 * max_deceleration_mps2 * ds[index]
        )
        speed[index] = min(speed[index], reachable)
    return speed.astype(np.float32)


__all__ = [
    "VelocityPlannerConfig",
    "build_mpc_path",
    "plan_curvature_speed_profile",
    "plan_velocity_profile",
]
