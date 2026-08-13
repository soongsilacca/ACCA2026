from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from multimodal_planner_v5.data import V5_TARGET_FIELDS, load_split_manifest


@dataclass(frozen=True)
class Candidate:
    split: str
    dataset_sample_index: int
    run_id: str
    sample_id: int
    gps_blackout: bool
    current_frame_idx: int
    pose_x: float
    pose_y: float
    pose_yaw: float
    speed_mps: float
    steering: float
    longitudinal_accel: float
    trajectory: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Expand hand-picked V5 samples with spatially, kinematically, and "
            "trajectory-similar validation samples."
        )
    )
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--source-summary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--split-manifest",
        type=Path,
        default=Path("multimodal_planner_v5/splits/v001.json"),
    )
    parser.add_argument(
        "--samples-per-anchor",
        type=int,
        default=5,
        help="total samples retained for each anchor, including the anchor",
    )
    parser.add_argument(
        "--minimum-sample-gap",
        type=int,
        default=4,
        help="minimum same-run sample-id gap between selected examples",
    )
    return parser.parse_args()


def _target(sample: Any) -> np.ndarray:
    if all(field in sample.files for field in V5_TARGET_FIELDS):
        return np.stack(
            [np.asarray(sample[field], dtype=np.float32) for field in V5_TARGET_FIELDS],
            axis=-1,
        )
    return np.concatenate(
        (
            np.asarray(sample["future_xy"], dtype=np.float32),
            np.asarray(sample["future_yaw"], dtype=np.float32)[..., None],
            np.asarray(sample["future_speed"], dtype=np.float32)[..., None],
        ),
        axis=-1,
    )


def _load_frame_arrays(run_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    manifest = json.loads((run_dir / "frame_chunks.json").read_text(encoding="utf-8"))
    entries = manifest.get("frame_chunks", manifest.get("chunks", manifest))
    pose_parts = []
    vehicle_parts = []
    for entry in entries:
        with np.load(run_dir / entry["file"], allow_pickle=False) as frame:
            pose_parts.append(np.asarray(frame["pose"], dtype=np.float64))
            vehicle_parts.append(np.asarray(frame["vehicle"], dtype=np.float32))
    return np.concatenate(pose_parts), np.concatenate(vehicle_parts)


def load_candidates(
    data_root: Path,
    split_manifest: Path,
    requested_splits: set[str],
) -> tuple[list[Candidate], dict[tuple[str, int], Candidate]]:
    splits = load_split_manifest(split_manifest)
    candidates: list[Candidate] = []
    lookup: dict[tuple[str, int], Candidate] = {}
    for split in requested_splits:
        dataset_index = 0
        for run_id in splits[split]:
            run_dir = data_root / run_id
            pose, vehicle = _load_frame_arrays(run_dir)
            with np.load(run_dir / "sample_index.npz", allow_pickle=False) as sample:
                targets = _target(sample)
                current = np.asarray(sample["current_frame_idx"], dtype=np.int64)
                sample_ids = np.asarray(sample["sample_id"], dtype=np.int64)
                blackout = np.asarray(sample["gps_blackout"], dtype=np.bool_)
                for local_index in range(len(sample_ids)):
                    frame_index = int(current[local_index])
                    item = Candidate(
                        split=split,
                        dataset_sample_index=dataset_index,
                        run_id=run_id,
                        sample_id=int(sample_ids[local_index]),
                        gps_blackout=bool(blackout[local_index]),
                        current_frame_idx=frame_index,
                        pose_x=float(pose[frame_index, 0]),
                        pose_y=float(pose[frame_index, 1]),
                        pose_yaw=float(pose[frame_index, 7]),
                        speed_mps=float(vehicle[frame_index, 0]),
                        steering=float(vehicle[frame_index, 3]),
                        longitudinal_accel=float(vehicle[frame_index, 4]),
                        trajectory=targets[local_index],
                    )
                    candidates.append(item)
                    lookup[(split, dataset_index)] = item
                    dataset_index += 1
    return candidates, lookup


def _angle_difference(left: float | np.ndarray, right: float | np.ndarray) -> Any:
    return np.arctan2(np.sin(left - right), np.cos(left - right))


def similarity(anchor: Candidate, candidate: Candidate) -> tuple[float, dict[str, float]]:
    position_m = math.hypot(
        candidate.pose_x - anchor.pose_x,
        candidate.pose_y - anchor.pose_y,
    )
    speed_mps = abs(candidate.speed_mps - anchor.speed_mps)
    steering = abs(candidate.steering - anchor.steering)
    acceleration = abs(candidate.longitudinal_accel - anchor.longitudinal_accel)
    yaw_rad = abs(float(_angle_difference(candidate.pose_yaw, anchor.pose_yaw)))

    sample_points = np.asarray([0, 4, 9, 14, 19])
    anchor_traj = anchor.trajectory[sample_points]
    candidate_traj = candidate.trajectory[sample_points]
    xy_rmse_m = float(
        np.sqrt(np.mean(np.square(candidate_traj[:, :2] - anchor_traj[:, :2])))
    )
    trajectory_yaw_rad = float(
        np.mean(
            np.abs(
                _angle_difference(candidate_traj[:, 2], anchor_traj[:, 2])
            )
        )
    )
    trajectory_speed_mps = float(
        np.mean(np.abs(candidate_traj[:, 3] - anchor_traj[:, 3]))
    )
    blackout_penalty = 2.0 if candidate.gps_blackout != anchor.gps_blackout else 0.0
    score = (
        position_m / 15.0
        + speed_mps / 4.0
        + steering / 0.15
        + acceleration / 2.5
        + yaw_rad / 0.35
        + xy_rmse_m / 4.0
        + trajectory_yaw_rad / 0.25
        + trajectory_speed_mps / 4.0
        + blackout_penalty
    )
    return score, {
        "position_m": position_m,
        "speed_mps": speed_mps,
        "steering": steering,
        "longitudinal_accel_mps2": acceleration,
        "ego_yaw_rad": yaw_rad,
        "trajectory_xy_rmse_m": xy_rmse_m,
        "trajectory_yaw_rad": trajectory_yaw_rad,
        "trajectory_speed_mps": trajectory_speed_mps,
        "score": score,
    }


def _far_enough(
    candidate: Candidate,
    selected: list[Candidate],
    minimum_sample_gap: int,
) -> bool:
    return all(
        candidate.run_id != other.run_id
        or abs(candidate.sample_id - other.sample_id) >= minimum_sample_gap
        for other in selected
    )


def main() -> None:
    args = parse_args()
    if args.samples_per_anchor < 1:
        raise ValueError("--samples-per-anchor must be positive")
    source = json.loads(args.source_summary.read_text(encoding="utf-8"))
    anchors = source.get("samples", [])
    if not anchors:
        raise ValueError(f"{args.source_summary}: no samples")

    requested_splits = {str(row["split"]) for row in anchors}
    candidates, candidate_lookup = load_candidates(
        args.data_root,
        args.split_manifest,
        requested_splits,
    )
    rows: list[dict[str, Any]] = []
    group_counts: dict[str, int] = {}

    for source_row in anchors:
        split = str(source_row["split"])
        anchor_index = int(source_row["dataset_sample_index"])
        anchor = candidate_lookup[(split, anchor_index)]
        ranked = sorted(
            (
                (similarity(anchor, candidate), candidate)
                for candidate in candidates
                if candidate.split == split
            ),
            key=lambda item: item[0][0],
        )
        selected = [anchor]
        selected_metrics = [
            {
                "position_m": 0.0,
                "speed_mps": 0.0,
                "steering": 0.0,
                "longitudinal_accel_mps2": 0.0,
                "ego_yaw_rad": 0.0,
                "trajectory_xy_rmse_m": 0.0,
                "trajectory_yaw_rad": 0.0,
                "trajectory_speed_mps": 0.0,
                "score": 0.0,
            }
        ]
        for (score, metrics), candidate in ranked:
            if candidate == anchor:
                continue
            if not _far_enough(candidate, selected, args.minimum_sample_gap):
                continue
            selected.append(candidate)
            selected_metrics.append(metrics)
            if len(selected) >= args.samples_per_anchor:
                break

        anchor_name = str(source_row.get("name", f"sample_{anchor_index}"))
        group = f"{source_row.get('category', 'representative')} / {anchor_name}"
        group_counts[group] = len(selected)
        for rank, (candidate, metrics) in enumerate(
            zip(selected, selected_metrics, strict=True)
        ):
            name = anchor_name if rank == 0 else f"{anchor_name}_similar_{rank:02d}"
            rows.append(
                {
                    "split": candidate.split,
                    "dataset_sample_index": candidate.dataset_sample_index,
                    "run_id": candidate.run_id,
                    "sample_id": candidate.sample_id,
                    "gps_blackout": candidate.gps_blackout,
                    "name": name,
                    "category": str(source_row.get("category", "representative")),
                    "group": group,
                    "anchor_name": anchor_name,
                    "anchor_dataset_sample_index": anchor_index,
                    "similarity_rank": rank,
                    "similarity": metrics,
                    "kinematic_state": {
                        "pose_x": candidate.pose_x,
                        "pose_y": candidate.pose_y,
                        "pose_yaw": candidate.pose_yaw,
                        "speed_mps": candidate.speed_mps,
                        "steering": candidate.steering,
                        "longitudinal_accel": candidate.longitudinal_accel,
                    },
                }
            )

    payload = {
        "schema_version": 1,
        "selection": {
            "method": "localization_kinematics_and_gt_trajectory_similarity",
            "samples_per_anchor": args.samples_per_anchor,
            "minimum_sample_gap": args.minimum_sample_gap,
            "source_summary": str(args.source_summary.resolve()),
            "group_counts": group_counts,
        },
        "sample_count": len(rows),
        "samples": rows,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "output": str(args.output),
                "sample_count": len(rows),
                "group_counts": group_counts,
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
