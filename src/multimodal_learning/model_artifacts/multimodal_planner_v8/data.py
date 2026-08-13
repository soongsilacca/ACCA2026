from __future__ import annotations

from typing import Any

import numpy as np
import torch

from multimodal_planner_v7.data import (
    AVOIDANCE_LATERAL_THRESHOLD_M,
    DRIVE,
    MOTION_STATE_NAMES,
    STOP,
    PlannerDataset as PlannerDatasetV7,
    deterministic_run_split,
    load_split_manifest,
    save_split_manifest,
)


SPATIAL_ANCHOR_STEP_M = 3.0
SPATIAL_ANCHOR_COUNT = 20
SPATIAL_ANCHORS_M = np.arange(
    1,
    SPATIAL_ANCHOR_COUNT + 1,
    dtype=np.float32,
) * SPATIAL_ANCHOR_STEP_M


def temporal_residual_to_spatial_np(
    progress_m: np.ndarray,
    lateral_m: np.ndarray,
    anchors_m: np.ndarray = SPATIAL_ANCHORS_M,
) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate unchanged temporal GT residuals onto fixed route stations.

    Stations beyond the distance actually reached by the four-second target are
    masked, rather than being fabricated as zero-valued labels.
    """
    progress = np.asarray(progress_m, dtype=np.float32).reshape(-1)
    lateral = np.asarray(lateral_m, dtype=np.float32).reshape(-1)
    anchors = np.asarray(anchors_m, dtype=np.float32).reshape(-1)
    if progress.shape != lateral.shape:
        raise ValueError("progress and lateral arrays must have the same shape")
    if not np.isfinite(progress).all() or not np.isfinite(lateral).all():
        raise ValueError("progress and lateral targets must be finite")
    if np.any(np.diff(anchors) <= 0.0):
        raise ValueError("anchors must be strictly increasing")

    # Add the exact local origin, then retain the furthest observation for each
    # monotonically increasing progress value. No source value is corrected.
    pairs = [(0.0, 0.0)]
    furthest = 0.0
    for s_value, d_value in zip(progress.tolist(), lateral.tolist()):
        if s_value < furthest - 1.0e-3 or s_value < 0.0:
            continue
        furthest = max(furthest, float(s_value))
        if abs(float(s_value) - pairs[-1][0]) <= 1.0e-3:
            pairs[-1] = (float(s_value), float(d_value))
        else:
            pairs.append((float(s_value), float(d_value)))

    valid = anchors <= furthest + 1.0e-3
    target = np.zeros_like(anchors, dtype=np.float32)
    if valid.any() and len(pairs) >= 2:
        source_s = np.asarray([item[0] for item in pairs], dtype=np.float32)
        source_d = np.asarray([item[1] for item in pairs], dtype=np.float32)
        target[valid] = np.interp(anchors[valid], source_s, source_d)
    return target, valid.astype(np.bool_)


class PlannerDataset(PlannerDatasetV7):
    """V7 raw inputs with distance-indexed, validity-masked Δd supervision."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        targets = []
        masks = []
        for progress, lateral in zip(
            self.route_progress_targets,
            self.lateral_residual_targets,
        ):
            target, mask = temporal_residual_to_spatial_np(progress, lateral)
            targets.append(target)
            masks.append(mask)
        self.spatial_lateral_targets = np.asarray(targets, dtype=np.float32)
        self.spatial_lateral_masks = np.asarray(masks, dtype=np.bool_)

    def __getitem__(self, item: int) -> dict[str, Any]:
        sample = super().__getitem__(item)
        sample["target_spatial_lateral_m"] = torch.from_numpy(
            self.spatial_lateral_targets[item].copy()
        )
        sample["target_spatial_valid"] = torch.from_numpy(
            self.spatial_lateral_masks[item].copy()
        )
        return sample

    def summary(self) -> dict[str, Any]:
        summary = super().summary()
        valid_per_anchor = self.spatial_lateral_masks.sum(axis=0)
        summary.update(
            {
                "architecture_target": "fixed_distance_lateral_residual_only",
                "spatial_anchors_m": SPATIAL_ANCHORS_M.tolist(),
                "valid_labels_per_anchor": valid_per_anchor.tolist(),
                "mean_valid_anchors_per_sample": float(
                    self.spatial_lateral_masks.sum(axis=1).mean()
                ),
                "label_policy": (
                    "source trajectory unchanged; temporal GT is projected and "
                    "interpolated only at reached spatial stations; unreached "
                    "stations are masked"
                ),
            }
        )
        return summary


__all__ = [
    "AVOIDANCE_LATERAL_THRESHOLD_M",
    "DRIVE",
    "MOTION_STATE_NAMES",
    "PlannerDataset",
    "SPATIAL_ANCHOR_COUNT",
    "SPATIAL_ANCHORS_M",
    "SPATIAL_ANCHOR_STEP_M",
    "STOP",
    "deterministic_run_split",
    "load_split_manifest",
    "save_split_manifest",
    "temporal_residual_to_spatial_np",
]

