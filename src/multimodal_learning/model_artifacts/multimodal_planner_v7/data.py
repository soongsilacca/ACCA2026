from __future__ import annotations

from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np
import torch

from multimodal_planner_v6.data import (
    DRIVE,
    MOTION_STATE_NAMES,
    STOP,
    STOP_MAX_ENDPOINT_DISTANCE_M,
    STOP_SPEED_THRESHOLD_MPS,
    PlannerDataset as PlannerDatasetV6,
    classify_motion_state,
    deterministic_run_split,
    load_split_manifest,
    save_split_manifest,
)
from multimodal_planner_v5.data import TARGET_SCALES


AVOIDANCE_LATERAL_THRESHOLD_M = 0.75


def project_target_to_route_np(
    route: np.ndarray,
    physical_target: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return target progress and signed path-normal residual without correction."""
    route_value = np.asarray(route, dtype=np.float32)
    target_value = np.asarray(physical_target, dtype=np.float32)
    if route_value.shape != (64, 4):
        raise ValueError(f"route must have shape [64,4], got {route_value.shape}")
    if target_value.shape != (20, 4):
        raise ValueError(
            f"physical_target must have shape [20,4], got {target_value.shape}"
        )
    points = route_value[:, :2]
    segments = points[1:] - points[:-1]
    lengths = np.maximum(np.linalg.norm(segments, axis=-1), 1.0e-4)
    cumulative = np.concatenate(
        (np.zeros(1, dtype=np.float32), np.cumsum(lengths, dtype=np.float32))
    )

    origin_delta = -points[:-1]
    origin_fraction = np.clip(
        np.sum(origin_delta * segments, axis=-1) / np.square(lengths),
        0.0,
        1.0,
    )
    origin_projected = points[:-1] + origin_fraction[:, None] * segments
    origin_segment = int(np.argmin(np.sum(np.square(origin_projected), axis=-1)))
    origin_s = (
        cumulative[origin_segment]
        + origin_fraction[origin_segment] * lengths[origin_segment]
    )

    target_xy = target_value[:, :2]
    delta = target_xy[:, None, :] - points[None, :-1, :]
    fraction = np.clip(
        np.sum(delta * segments[None, :, :], axis=-1)
        / np.square(lengths)[None, :],
        0.0,
        1.0,
    )
    projected = points[None, :-1, :] + fraction[..., None] * segments[None]
    residual = target_xy[:, None, :] - projected
    closest = np.argmin(np.sum(np.square(residual), axis=-1), axis=1)
    row = np.arange(target_xy.shape[0])
    progress = (
        cumulative[closest]
        + fraction[row, closest] * lengths[closest]
        - origin_s
    )
    unit = segments[closest] / lengths[closest, None]
    selected_residual = residual[row, closest]
    lateral = (
        unit[:, 0] * selected_residual[:, 1]
        - unit[:, 1] * selected_residual[:, 0]
    )
    return progress.astype(np.float32), lateral.astype(np.float32)


class PlannerDataset(PlannerDatasetV6):
    """V6 inputs plus GT residuals relative to the unmodified Local Route."""

    def __init__(
        self,
        data_root: Path,
        run_ids: list[str],
        blackout_weight: float = 2.0,
        stop_weight: float = 4.0,
        drive_weight: float = 1.0,
        avoidance_weight: float = 6.0,
        avoidance_threshold_m: float = AVOIDANCE_LATERAL_THRESHOLD_M,
        max_samples: int = 0,
        seed: int = 2026,
        allow_legacy_target_fields: bool = False,
        photometric_augmentation: bool = False,
        stop_speed_threshold_mps: float = STOP_SPEED_THRESHOLD_MPS,
        stop_max_endpoint_distance_m: float = STOP_MAX_ENDPOINT_DISTANCE_M,
    ) -> None:
        if avoidance_weight <= 0.0 or avoidance_threshold_m <= 0.0:
            raise ValueError("avoidance weight and threshold must be positive")
        super().__init__(
            data_root=data_root,
            run_ids=run_ids,
            blackout_weight=blackout_weight,
            stop_weight=stop_weight,
            drive_weight=drive_weight,
            max_samples=max_samples,
            seed=seed,
            allow_legacy_target_fields=allow_legacy_target_fields,
            photometric_augmentation=photometric_augmentation,
            stop_speed_threshold_mps=stop_speed_threshold_mps,
            stop_max_endpoint_distance_m=stop_max_endpoint_distance_m,
        )
        self.avoidance_weight = float(avoidance_weight)
        self.avoidance_threshold_m = float(avoidance_threshold_m)
        progress_targets = []
        lateral_targets = []
        avoidance_labels = []
        final_weights = []
        for item, (run_index, sample_index) in enumerate(self.lookup):
            run = self.runs[run_index]
            frame_index = int(run.current_frame_idx[sample_index])
            chunk_index = run._chunk_index(frame_index)
            chunk_info = run.chunks[chunk_index]
            chunk = run._load_chunk(chunk_index)
            route = np.asarray(
                chunk["route"][frame_index - chunk_info.start],
                dtype=np.float32,
            )
            progress, lateral = project_target_to_route_np(
                route,
                run.target[sample_index],
            )
            avoidance = bool(np.max(np.abs(lateral)) >= self.avoidance_threshold_m)
            progress_targets.append(progress)
            lateral_targets.append(lateral)
            avoidance_labels.append(avoidance)
            final_weights.append(
                float(self.combined_sample_weights[item])
                * (self.avoidance_weight if avoidance else 1.0)
            )
        self.route_progress_targets = np.asarray(
            progress_targets,
            dtype=np.float32,
        )
        self.lateral_residual_targets = np.asarray(
            lateral_targets,
            dtype=np.float32,
        )
        self.avoidance_labels = np.asarray(avoidance_labels, dtype=np.bool_)
        self.final_sample_weights = np.asarray(final_weights, dtype=np.float32)
        self.avoidance_count = int(self.avoidance_labels.sum())
        self._mean_final_sample_weight = float(
            self.final_sample_weights.mean()
            if len(self.final_sample_weights)
            else 1.0
        )

    @property
    def mean_sample_weight(self) -> float:
        if hasattr(self, "_mean_final_sample_weight"):
            return self._mean_final_sample_weight
        return super().mean_sample_weight

    def __getitem__(self, item: int) -> dict[str, Any]:
        sample = super().__getitem__(item)
        sample["sample_weight"] = torch.tensor(
            float(self.final_sample_weights[item]),
            dtype=torch.float32,
        )
        sample["target_route_progress_m"] = torch.from_numpy(
            self.route_progress_targets[item].copy()
        )
        sample["target_lateral_residual_m"] = torch.from_numpy(
            self.lateral_residual_targets[item].copy()
        )
        sample["avoidance"] = torch.tensor(bool(self.avoidance_labels[item]))
        return sample

    def summary(self) -> dict[str, Any]:
        summary = super().summary()
        abs_lateral = np.abs(self.lateral_residual_targets)
        summary.update(
            {
                "architecture_target": (
                    "future_speed_plus_local_route_lateral_residual"
                ),
                "avoidance_threshold_m": self.avoidance_threshold_m,
                "avoidance_weight": self.avoidance_weight,
                "avoidance_count": self.avoidance_count,
                "non_avoidance_count": len(self) - self.avoidance_count,
                "avoidance_fraction": self.avoidance_count / max(len(self), 1),
                "lateral_residual_abs_percentiles_m": {
                    str(percentile): float(np.percentile(abs_lateral, percentile))
                    for percentile in (50, 90, 95, 99, 100)
                },
                "mean_final_sample_weight": self.mean_sample_weight,
                "label_policy": (
                    "original target values unchanged; route progress and signed "
                    "path-normal residual are deterministic derived supervision"
                ),
            }
        )
        return summary

__all__ = [
    "DRIVE",
    "MOTION_STATE_NAMES",
    "STOP",
    "STOP_MAX_ENDPOINT_DISTANCE_M",
    "STOP_SPEED_THRESHOLD_MPS",
    "PlannerDataset",
    "AVOIDANCE_LATERAL_THRESHOLD_M",
    "classify_motion_state",
    "deterministic_run_split",
    "load_split_manifest",
    "save_split_manifest",
    "project_target_to_route_np",
]
