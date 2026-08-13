from __future__ import annotations

import argparse
import gc
import json
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from multimodal_planner_v7.data import (
    MOTION_STATE_NAMES,
    PlannerDataset,
    load_split_manifest,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Mine multi-frame large-Δd, STOP and known obstacle events across bags."
        )
    )
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--split-manifest", type=Path, required=True)
    parser.add_argument("--obstacle-source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--large-delta-threshold-m", type=float, default=0.75)
    parser.add_argument("--event-gap-samples", type=int, default=6)
    parser.add_argument("--max-events-per-run", type=int, default=4)
    parser.add_argument(
        "--context-offsets",
        type=int,
        nargs="+",
        default=(-12, -8, -4, 0, 4, 8, 12),
    )
    return parser.parse_args()


def _events(
    candidates: list[tuple[int, int]],
    maximum_gap: int,
) -> list[list[tuple[int, int]]]:
    if not candidates:
        return []
    ordered = sorted(candidates, key=lambda value: value[1])
    groups = [[ordered[0]]]
    for candidate in ordered[1:]:
        if candidate[1] - groups[-1][-1][1] <= maximum_gap:
            groups[-1].append(candidate)
        else:
            groups.append([candidate])
    return groups


def _event_centers(
    event: list[tuple[int, int]],
) -> list[tuple[str, int]]:
    positions = [value[1] for value in event]
    return [
        ("entry", positions[0]),
        ("quarter", positions[len(positions) // 4]),
        ("middle", positions[len(positions) // 2]),
        ("three_quarter", positions[(3 * len(positions)) // 4]),
        ("exit", positions[-1]),
    ]


def _sample_lookup(
    dataset: PlannerDataset,
) -> tuple[
    dict[str, list[tuple[int, int]]],
    dict[str, dict[int, int]],
]:
    by_run: defaultdict[str, list[tuple[int, int]]] = defaultdict(list)
    by_sample: defaultdict[str, dict[int, int]] = defaultdict(dict)
    for dataset_index, (run_index, local_index) in enumerate(dataset.lookup):
        run = dataset.runs[run_index]
        sample_id = int(run.sample_id[local_index])
        by_run[run.run_id].append((dataset_index, sample_id))
        by_sample[run.run_id][sample_id] = dataset_index
    return dict(by_run), dict(by_sample)


def _nearest_index(
    sample_map: dict[int, int],
    requested_sample_id: int,
) -> tuple[int, int]:
    nearest = min(sample_map, key=lambda value: abs(value - requested_sample_id))
    return sample_map[nearest], nearest


def _row(
    dataset: PlannerDataset,
    split: str,
    dataset_index: int,
    criterion: str,
    event_name: str,
    phase: str,
) -> dict[str, Any]:
    run_index, local_index = dataset.lookup[dataset_index]
    run = dataset.runs[run_index]
    sample_id = int(run.sample_id[local_index])
    target_delta = float(
        np.max(np.abs(dataset.lateral_residual_targets[dataset_index]))
    )
    motion_state = int(dataset.motion_state_labels[dataset_index])
    return {
        "split": split,
        "dataset_sample_index": dataset_index,
        "run_id": run.run_id,
        "sample_id": sample_id,
        "gps_blackout": bool(run.gps_blackout[local_index]),
        "name": f"{criterion}_{event_name}_{phase}_{sample_id:05d}",
        "category": criterion,
        "group": f"{criterion} / {run.run_id} / {event_name}",
        "anchor_name": event_name,
        "anchor_dataset_sample_index": dataset_index,
        "similarity_rank": 0,
        "similarity": {"score": 0.0},
        "event_review": {
            "criterion": criterion,
            "phase": phase,
            "target_max_abs_lateral_residual_m": target_delta,
            "motion_state": MOTION_STATE_NAMES[motion_state],
        },
    }


def _mine_boolean_events(
    dataset: PlannerDataset,
    split: str,
    criterion: str,
    mask: np.ndarray,
    event_gap: int,
    max_events_per_run: int,
    context_offsets: Iterable[int],
) -> list[dict[str, Any]]:
    by_run, by_sample = _sample_lookup(dataset)
    candidates: defaultdict[str, list[tuple[int, int]]] = defaultdict(list)
    for run_id, values in by_run.items():
        for dataset_index, sample_id in values:
            if bool(mask[dataset_index]):
                candidates[run_id].append((dataset_index, sample_id))
    rows = []
    for run_id, run_candidates in candidates.items():
        run_events = _events(run_candidates, event_gap)
        # Prefer longer events, then restore temporal order for browsing.
        selected = sorted(
            sorted(run_events, key=len, reverse=True)[:max_events_per_run],
            key=lambda event: event[0][1],
        )
        sample_map = by_sample[run_id]
        for event_index, event in enumerate(selected, 1):
            event_name = f"event_{event_index:02d}"
            requested: list[tuple[str, int]] = []
            requested.extend(_event_centers(event))
            midpoint = event[len(event) // 2][1]
            requested.extend(
                (f"context_{offset:+d}", midpoint + int(offset))
                for offset in context_offsets
            )
            used: set[int] = set()
            for phase, requested_id in requested:
                dataset_index, actual_id = _nearest_index(sample_map, requested_id)
                if actual_id in used:
                    continue
                used.add(actual_id)
                rows.append(
                    _row(
                        dataset,
                        split,
                        dataset_index,
                        criterion,
                        event_name,
                        phase,
                    )
                )
    return rows


def _expand_obstacle_seeds(
    dataset: PlannerDataset,
    split: str,
    obstacle_source: Path,
    context_offsets: Iterable[int],
    event_gap: int,
) -> list[dict[str, Any]]:
    source = json.loads(obstacle_source.read_text(encoding="utf-8"))
    seeds: defaultdict[str, list[tuple[int, int]]] = defaultdict(list)
    for row in source.get("samples", []):
        text = f"{row.get('group', '')} {row.get('category', '')}".lower()
        if "obstacle" not in text or str(row["split"]) != split:
            continue
        seeds[str(row["run_id"])].append(
            (int(row["dataset_sample_index"]), int(row["sample_id"]))
        )
    rows = []
    sample_lookup = _sample_lookup(dataset)[1]
    for run_id, candidates in seeds.items():
        if run_id not in sample_lookup:
            continue
        sample_map = sample_lookup[run_id]
        for event_index, event in enumerate(_events(candidates, event_gap * 5), 1):
            center = event[len(event) // 2][1]
            used: set[int] = set()
            for offset in context_offsets:
                dataset_index, actual_id = _nearest_index(
                    sample_map,
                    center + int(offset),
                )
                if actual_id in used:
                    continue
                used.add(actual_id)
                rows.append(
                    _row(
                        dataset,
                        split,
                        dataset_index,
                        "obstacle",
                        f"event_{event_index:02d}",
                        f"context_{int(offset):+d}",
                    )
                )
    return rows


def main() -> None:
    args = parse_args()
    splits = load_split_manifest(args.split_manifest)
    rows: list[dict[str, Any]] = []
    for split, run_ids in splits.items():
        dataset = PlannerDataset(
            args.data_root,
            run_ids,
            blackout_weight=1.0,
            stop_weight=1.0,
            drive_weight=1.0,
            avoidance_weight=1.0,
            avoidance_threshold_m=args.large_delta_threshold_m,
            allow_legacy_target_fields=True,
            photometric_augmentation=False,
        )
        large_delta = (
            np.max(np.abs(dataset.lateral_residual_targets), axis=1)
            >= args.large_delta_threshold_m
        )
        stop = dataset.motion_state_labels == 0
        rows.extend(
            _mine_boolean_events(
                dataset,
                split,
                "large_delta",
                large_delta,
                args.event_gap_samples,
                args.max_events_per_run,
                args.context_offsets,
            )
        )
        rows.extend(
            _mine_boolean_events(
                dataset,
                split,
                "stopped",
                stop,
                args.event_gap_samples,
                args.max_events_per_run,
                args.context_offsets,
            )
        )
        rows.extend(
            _expand_obstacle_seeds(
                dataset,
                split,
                args.obstacle_source,
                args.context_offsets,
                args.event_gap_samples,
            )
        )
        del dataset
        gc.collect()
    # Preserve criterion/event order but remove identical criterion frames.
    deduplicated = []
    seen: set[tuple[str, str, int]] = set()
    for row in rows:
        key = (
            str(row["category"]),
            str(row["split"]),
            int(row["dataset_sample_index"]),
        )
        if key in seen:
            continue
        seen.add(key)
        deduplicated.append(row)
    counts: defaultdict[str, int] = defaultdict(int)
    runs: defaultdict[str, set[str]] = defaultdict(set)
    for row in deduplicated:
        counts[str(row["category"])] += 1
        runs[str(row["category"])].add(str(row["run_id"]))
    payload = {
        "schema_version": 1,
        "selection": {
            "policy": "multi-frame continuous event mining",
            "large_delta_threshold_m": args.large_delta_threshold_m,
            "event_gap_samples": args.event_gap_samples,
            "max_events_per_run": args.max_events_per_run,
            "context_offsets": list(args.context_offsets),
            "source_values_corrected": False,
        },
        "sample_count": len(deduplicated),
        "counts": dict(counts),
        "run_counts": {name: len(value) for name, value in runs.items()},
        "samples": deduplicated,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "complete": True,
                "output": str(args.output),
                "sample_count": len(deduplicated),
                "counts": dict(counts),
                "run_counts": {name: len(value) for name, value in runs.items()},
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
