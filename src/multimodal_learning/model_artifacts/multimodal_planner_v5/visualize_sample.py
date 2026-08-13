from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import torch

from multimodal_planner_v5.data import PlannerDataset, load_split_manifest
from multimodal_planner_v5.model import (
    ModelConfig,
    MultiViewTemporalTrajectoryPlannerV5,
)
from multimodal_planner_v5.outliers import (
    TopOutlierCollector,
    trajectory_diagnostics,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument(
        "--split-manifest",
        type=Path,
        default=Path("multimodal_planner_v5/splits/v001.json"),
    )
    parser.add_argument("--split", choices=("train", "val", "test"), default="val")
    parser.add_argument("--sample-index", type=int, default=0)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--allow-legacy-target-fields", action="store_true")
    parser.add_argument("--cpu", action="store_true")
    return parser.parse_args()


def make_batch(sample: dict[str, Any], device: torch.device) -> dict[str, Any]:
    batch: dict[str, Any] = {}
    for key, value in sample.items():
        if isinstance(value, torch.Tensor):
            batch[key] = value.unsqueeze(0).to(device)
        elif key == "run_id":
            batch[key] = [value]
        elif key == "sample_id":
            batch[key] = torch.tensor([value], device=device)
        else:
            batch[key] = value
    return batch


def main() -> None:
    args = parse_args()
    splits = load_split_manifest(args.split_manifest)
    dataset = PlannerDataset(
        args.data_root,
        splits[args.split],
        blackout_weight=1.0,
        allow_legacy_target_fields=args.allow_legacy_target_fields,
    )
    if not 0 <= args.sample_index < len(dataset):
        raise IndexError(
            f"sample-index {args.sample_index} is outside [0, {len(dataset)})"
        )
    device = torch.device(
        "cpu" if args.cpu or not torch.cuda.is_available() else "cuda"
    )
    checkpoint = torch.load(
        args.checkpoint,
        map_location=device,
        weights_only=False,
    )
    config = ModelConfig(**checkpoint["model_config"])
    model = MultiViewTemporalTrajectoryPlannerV5(config).to(device)
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    batch = make_batch(dataset[args.sample_index], device)
    with torch.inference_mode(), torch.autocast(
        device_type=device.type,
        dtype=torch.float16,
        enabled=device.type == "cuda",
    ):
        outputs = model(
            batch["front"],
            batch["left"],
            batch["right"],
            batch["lidar_bev"],
            batch["ego"],
            batch["mgeo"],
            batch["local_route"],
        )
        diagnostics = trajectory_diagnostics(outputs, batch["target"])

    collector = TopOutlierCollector(1)
    collector.update(outputs, batch, diagnostics)
    summary = collector.export(args.output_dir, int(checkpoint["epoch"]) + 1)
    result = {
        "checkpoint": str(args.checkpoint.resolve()),
        "checkpoint_epoch": int(checkpoint["epoch"]) + 1,
        "split": args.split,
        "dataset_sample_index": args.sample_index,
        "run_id": batch["run_id"][0],
        "sample_id": int(batch["sample_id"][0]),
        "gps_blackout": bool(batch["gps_blackout"][0]),
        "ade_m": float(diagnostics["ade_m"][0]),
        "fde_m": float(diagnostics["fde_m"][0]),
        "yaw_mae_deg": float(diagnostics["yaw_mae_deg"][0]),
        "longitudinal_mae_m": float(diagnostics["longitudinal_mae_m"][0]),
        "lateral_mae_m": float(diagnostics["lateral_mae_m"][0]),
        "step_mae_m": float(diagnostics["step_mae_m"][0]),
        "acceleration_mae_m": float(diagnostics["acceleration_mae_m"][0]),
        "heading_consistency_mae_deg": float(
            diagnostics["heading_consistency_mae_deg"][0]
        ),
        "first_waypoint_error_m": float(
            diagnostics["first_waypoint_error_m"][0]
        ),
        "predicted_first_x_m": float(diagnostics["predicted_first_x_m"][0]),
        "target_first_x_m": float(diagnostics["target_first_x_m"][0]),
        "origin_direction_mismatch": bool(
            diagnostics["origin_direction_mismatch"][0]
        ),
        "gt_relative_lateral_sign_flip_count": float(
            diagnostics["gt_relative_lateral_sign_flip_count"][0]
        ),
        "gt_relative_lateral_sign_flip_rate": float(
            diagnostics["gt_relative_lateral_sign_flip_rate"][0]
        ),
        "horizon_metrics": {
            f"{seconds}s": {
                "ade_m": float(diagnostics[f"ade_{seconds}s_m"][0]),
                "fde_m": float(diagnostics[f"fde_{seconds}s_m"][0]),
            }
            for seconds in (1, 2, 4)
        },
        "gallery": summary["gallery"],
        "manifest": summary["manifest"],
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    result_path = args.output_dir / "sample_result.json"
    result["result"] = str(result_path.resolve())
    result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
