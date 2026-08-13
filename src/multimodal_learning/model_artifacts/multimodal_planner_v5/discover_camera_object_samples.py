from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torchvision.models.detection import (
    SSDLite320_MobileNet_V3_Large_Weights,
    ssdlite320_mobilenet_v3_large,
)

from multimodal_planner_v5.data import PlannerDataset, load_split_manifest


OBJECT_CLASSES = {"person", "bicycle", "car", "motorcycle", "bus", "truck"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Find camera frames containing potential dynamic road users and merge "
            "them into a V5 sample-set JSON. Detections are candidates, not proof "
            "that the object is moving."
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
    parser.add_argument("--split", default="val")
    parser.add_argument("--stride", type=int, default=3)
    parser.add_argument("--score-threshold", type=float, default=0.25)
    parser.add_argument("--minimum-box-area-ratio", type=float, default=0.002)
    parser.add_argument("--max-candidates", type=int, default=24)
    parser.add_argument("--minimum-sample-gap", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=12)
    parser.add_argument("--threads", type=int, default=2)
    return parser.parse_args()


def _image_tensor(image: np.ndarray) -> torch.Tensor:
    return torch.from_numpy(
        np.ascontiguousarray(image.transpose(2, 0, 1))
    ).float().div_(255.0)


def _detections(
    outputs: list[dict[str, torch.Tensor]],
    image_meta: list[dict[str, Any]],
    categories: list[str],
    score_threshold: float,
    minimum_box_area_ratio: float,
) -> list[dict[str, Any]]:
    rows = []
    for output, meta in zip(outputs, image_meta, strict=True):
        height = int(meta["height"])
        width = int(meta["width"])
        image_area = float(height * width)
        matches = []
        for label, score, box in zip(
            output["labels"].tolist(),
            output["scores"].tolist(),
            output["boxes"].tolist(),
            strict=True,
        ):
            if score < score_threshold:
                break
            class_name = categories[int(label)]
            if class_name not in OBJECT_CLASSES:
                continue
            x1, y1, x2, y2 = map(float, box)
            area_ratio = max(0.0, x2 - x1) * max(0.0, y2 - y1) / image_area
            if area_ratio < minimum_box_area_ratio:
                continue
            matches.append(
                {
                    "class": class_name,
                    "score": float(score),
                    "box_xyxy": [x1, y1, x2, y2],
                    "box_area_ratio": area_ratio,
                    "priority": float(score) * math.sqrt(area_ratio),
                }
            )
        if matches:
            matches.sort(key=lambda item: item["priority"], reverse=True)
            rows.append({**meta, "detections": matches, "best": matches[0]})
    return rows


def _select_diverse(
    candidates: list[dict[str, Any]],
    max_candidates: int,
    minimum_sample_gap: int,
) -> list[dict[str, Any]]:
    selected: list[dict[str, Any]] = []
    per_class: defaultdict[str, int] = defaultdict(int)
    detected_classes = {str(item["best"]["class"]) for item in candidates}
    class_limit = max(
        3,
        math.ceil(max_candidates / max(1, len(detected_classes))),
    )
    for candidate in sorted(
        candidates,
        key=lambda item: item["best"]["priority"],
        reverse=True,
    ):
        class_name = str(candidate["best"]["class"])
        if per_class[class_name] >= class_limit:
            continue
        if any(
            candidate["run_id"] == other["run_id"]
            and abs(candidate["sample_id"] - other["sample_id"])
            < minimum_sample_gap
            for other in selected
        ):
            continue
        selected.append(candidate)
        per_class[class_name] += 1
        if len(selected) >= max_candidates:
            break
    return selected


def main() -> None:
    args = parse_args()
    if args.stride < 1 or args.batch_size < 1:
        raise ValueError("--stride and --batch-size must be positive")
    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)

    weights = SSDLite320_MobileNet_V3_Large_Weights.DEFAULT
    categories = list(weights.meta["categories"])
    model = ssdlite320_mobilenet_v3_large(weights=weights).eval()
    splits = load_split_manifest(args.split_manifest)
    dataset = PlannerDataset(
        args.data_root,
        splits[args.split],
        blackout_weight=1.0,
        allow_legacy_target_fields=True,
        photometric_augmentation=False,
    )

    pending_images: list[torch.Tensor] = []
    pending_meta: list[dict[str, Any]] = []
    detected: list[dict[str, Any]] = []

    def flush() -> None:
        if not pending_images:
            return
        with torch.inference_mode():
            outputs = model(pending_images)
        detected.extend(
            _detections(
                outputs,
                pending_meta,
                categories,
                args.score_threshold,
                args.minimum_box_area_ratio,
            )
        )
        pending_images.clear()
        pending_meta.clear()

    scanned_samples = 0
    for dataset_index in range(0, len(dataset), args.stride):
        run_index, local_index = dataset.lookup[dataset_index]
        run = dataset.runs[run_index]
        frame_index = int(run.current_frame_idx[local_index])
        frame = run.frame(frame_index)
        scanned_samples += 1
        for camera in ("front", "left", "right"):
            image = frame[camera]
            pending_images.append(_image_tensor(image))
            pending_meta.append(
                {
                    "split": args.split,
                    "dataset_sample_index": dataset_index,
                    "run_id": run.run_id,
                    "sample_id": int(run.sample_id[local_index]),
                    "gps_blackout": bool(run.gps_blackout[local_index]),
                    "current_frame_idx": frame_index,
                    "camera": camera,
                    "height": int(image.shape[0]),
                    "width": int(image.shape[1]),
                }
            )
            if len(pending_images) >= args.batch_size:
                flush()
        if scanned_samples % 100 == 0:
            print(
                json.dumps(
                    {
                        "scanned_samples": scanned_samples,
                        "detected_views": len(detected),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
    flush()

    source = json.loads(args.source_summary.read_text(encoding="utf-8"))
    rows = list(source.get("samples", []))
    existing = {
        (str(row["split"]), int(row["dataset_sample_index"])) for row in rows
    }
    selectable = [
        candidate
        for candidate in detected
        if (
            str(candidate["split"]),
            int(candidate["dataset_sample_index"]),
        )
        not in existing
    ]
    selected = _select_diverse(
        selectable,
        args.max_candidates,
        args.minimum_sample_gap,
    )
    class_ranks: defaultdict[str, int] = defaultdict(int)
    added = []
    for candidate in selected:
        key = (str(candidate["split"]), int(candidate["dataset_sample_index"]))
        if key in existing:
            continue
        class_name = str(candidate["best"]["class"])
        class_ranks[class_name] += 1
        rank = class_ranks[class_name]
        row = {
            "split": candidate["split"],
            "dataset_sample_index": candidate["dataset_sample_index"],
            "run_id": candidate["run_id"],
            "sample_id": candidate["sample_id"],
            "gps_blackout": candidate["gps_blackout"],
            "name": f"camera_{class_name}_{rank:02d}",
            "category": "dynamic_obstacle_candidate",
            "group": f"dynamic-obstacle candidates / {class_name}",
            "anchor_name": f"camera_{class_name}",
            "anchor_dataset_sample_index": candidate["dataset_sample_index"],
            "similarity_rank": rank - 1,
            "similarity": {"score": 0.0},
            "camera_detection": {
                "detector": "ssdlite320_mobilenet_v3_large_coco",
                "camera": candidate["camera"],
                "moving_object_confirmed": False,
                "candidate_only": True,
                "detections": candidate["detections"],
            },
        }
        rows.append(row)
        added.append(row)
        existing.add(key)

    payload = {
        **source,
        "sample_count": len(rows),
        "samples": rows,
        "camera_object_discovery": {
            "detector": "ssdlite320_mobilenet_v3_large_coco",
            "interpretation": (
                "camera object candidates only; motion is not confirmed by the "
                "single-frame detector"
            ),
            "split": args.split,
            "stride": args.stride,
            "scanned_samples": scanned_samples,
            "detected_views": len(detected),
            "selected_before_deduplication": len(selected),
            "added_samples": len(added),
            "score_threshold": args.score_threshold,
            "minimum_box_area_ratio": args.minimum_box_area_ratio,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "complete": True,
                "output": str(args.output),
                "total_samples": len(rows),
                "camera_candidates_added": len(added),
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
