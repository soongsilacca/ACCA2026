from __future__ import annotations

import argparse
import html
import json
from pathlib import Path
from typing import Any

import numpy as np
import torch
from PIL import Image, ImageDraw

from multimodal_planner_v5.visualize_sample import make_batch
from multimodal_planner_v7.recheck_samples import (
    _camera_image,
    _fit,
    _font,
    _panel,
    _plot_xy,
    _safe_name,
)
from multimodal_planner_v8.data import MOTION_STATE_NAMES, PlannerDataset, load_split_manifest
from multimodal_planner_v8.model import ModelConfig, SpatialResidualPlannerV8


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--source-summary", type=Path, required=True)
    parser.add_argument("--verified-obstacle-manifest", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--split-manifest", type=Path, required=True)
    parser.add_argument("--allow-legacy-target-fields", action="store_true")
    parser.add_argument("--max-samples", type=int, default=0)
    parser.add_argument("--cpu", action="store_true")
    return parser.parse_args()


def _state_outcome(target: str, prediction: str) -> str:
    if target == "STOP":
        return "TP" if prediction == "STOP" else "FN"
    return "FP" if prediction == "STOP" else "TN"


def _validation_state_summary(checkpoint: dict[str, Any]) -> dict[str, Any]:
    history = checkpoint.get("history", [])
    metrics = (
        history[-1].get("val", {}).get("motion_state_metrics", {})
        if history
        else {}
    )
    matrix = metrics.get("confusion_matrix_actual_rows_predicted_columns")
    if not matrix or len(matrix) != 2:
        return {}
    tp, fn = (int(value) for value in matrix[0])
    fp, tn = (int(value) for value in matrix[1])
    drive_count = fp + tn
    stop_count = tp + fn
    return {
        **metrics,
        "positive_class": "STOP",
        "tp": tp,
        "fn": fn,
        "fp": fp,
        "tn": tn,
        "false_positive_rate": fp / drive_count if drive_count else 0.0,
        "false_negative_rate": fn / stop_count if stop_count else 0.0,
    }


def _merge_verified_obstacles(
    requested: list[dict[str, Any]],
    manifest_path: Path | None,
) -> list[dict[str, Any]]:
    if manifest_path is None:
        return requested
    verified = json.loads(manifest_path.read_text(encoding="utf-8")).get("samples", [])
    merged = []
    for source_row in requested:
        row = dict(source_row)
        group = str(row.get("group", ""))
        if (
            row.get("category") == "dynamic_obstacle_candidate"
            or group.startswith("dynamic-obstacle candidates /")
            or group.startswith("obstacle /")
        ):
            row["group"] = f"unverified object/traffic candidate / {group}"
            row["category"] = "unverified_object_candidate"
            row["obstacle_kind"] = "none"
            row["obstacle_verification"] = "not_verified"
        merged.append(row)
    identity_to_index = {
        (str(row["split"]), str(row["run_id"]), int(row["sample_id"])): index
        for index, row in enumerate(merged)
    }
    for row in verified:
        identity = (str(row["split"]), str(row["run_id"]), int(row["sample_id"]))
        if identity in identity_to_index:
            merged[identity_to_index[identity]].update(row)
        else:
            identity_to_index[identity] = len(merged)
            merged.append(dict(row))
    return merged


def _plot_residual(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    stations: np.ndarray,
    target: np.ndarray,
    valid: np.ndarray,
    prediction: np.ndarray,
) -> None:
    x0, y0, x1, y1 = _panel(draw, box, "Fixed-distance Frenet residual Δd(s)")
    observed = np.concatenate((prediction, target[valid] if valid.any() else np.zeros(1)))
    ymin, ymax = float(observed.min()), float(observed.max())
    pad = max((ymax - ymin) * 0.15, 0.25)
    ymin, ymax = ymin - pad, ymax + pad
    draw.line((x0, y1, x1, y1), fill="#475569", width=1)
    if ymin <= 0 <= ymax:
        zy = y1 - (0 - ymin) / (ymax - ymin) * (y1 - y0)
        draw.line((x0, zy, x1, zy), fill="#334155", width=1)

    def points(values: np.ndarray, mask: np.ndarray | None = None) -> list[tuple[float, float]]:
        selected_s = stations if mask is None else stations[mask]
        selected_v = values if mask is None else values[mask]
        return [
            (
                x0 + float(s) / float(stations[-1]) * (x1 - x0),
                y1 - (float(v) - ymin) / (ymax - ymin) * (y1 - y0),
            )
            for s, v in zip(selected_s, selected_v)
        ]

    pred_points = points(prediction)
    if len(pred_points) >= 2:
        draw.line(pred_points, fill="#ef4444", width=5)
    target_points = points(target, valid)
    if len(target_points) >= 2:
        draw.line(target_points, fill="#22c55e", width=5)
    for point in target_points:
        draw.ellipse((point[0] - 3, point[1] - 3, point[0] + 3, point[1] + 3), fill="#22c55e")
    draw.text((x0, y1 + 8), "0m", font=_font(14), fill="#94a3b8")
    draw.text((x1 - 45, y1 + 8), "60m", font=_font(14), fill="#94a3b8")
    draw.line((x0 + 110, y1 + 20, x0 + 140, y1 + 20), fill="#22c55e", width=4)
    draw.text((x0 + 148, y1 + 10), "GT valid", font=_font(14), fill="#e2e8f0")
    draw.line((x0 + 250, y1 + 20, x0 + 280, y1 + 20), fill="#ef4444", width=4)
    draw.text((x0 + 288, y1 + 10), "Pred", font=_font(14), fill="#e2e8f0")


def _render(
    path: Path,
    batch: dict[str, Any],
    outputs: dict[str, torch.Tensor],
    result: dict[str, Any],
) -> None:
    canvas = Image.new("RGB", (1600, 1120), "#070b12")
    draw = ImageDraw.Draw(canvas)
    canvas.paste(_fit(_camera_image(batch["front"][0]), (760, 340)), (20, 42))
    canvas.paste(_fit(_camera_image(batch["left"][0]), (380, 280)), (800, 42))
    canvas.paste(_fit(_camera_image(batch["right"][0]), (380, 280)), (1200, 42))
    for x, name in ((20, "Front"), (800, "Left"), (1200, "Right")):
        draw.text((x, 12), name, font=_font(20), fill="#f8fafc")
    route = batch["local_route"][0, :, :2].detach().float().cpu().numpy() * 50.0
    target = batch["target"][0, :, :2].detach().float().cpu().numpy() * 50.0
    prediction = outputs["spatial_path_xy_m"][0].detach().float().cpu().numpy()
    _plot_xy(draw, (20, 405, 780, 1090), route, target, prediction)
    stations = outputs["spatial_stations_m"][0].detach().float().cpu().numpy()
    target_d = batch["target_spatial_lateral_m"][0].detach().float().cpu().numpy()
    valid = batch["target_spatial_valid"][0].detach().bool().cpu().numpy()
    pred_d = outputs["lateral_residual_m"][0].detach().float().cpu().numpy()
    _plot_residual(draw, (800, 405, 1580, 800), stations, target_d, valid, pred_d)
    lines = [
        f"#{result['dataset_sample_index']} {result['name']} | {result['group']}",
        f"{result['run_id']} / sample {result['sample_id']} | blackout={result['gps_blackout']} | avoidance={result['avoidance']}",
        f"state GT={result['motion_state_target']}  Pred={result['motion_state_prediction']} ({result['motion_state_confidence']:.3f})",
        f"valid anchors={result['valid_anchor_count']}/20 | farthest supervised station={result['farthest_valid_station_m']:.1f}m",
        f"spatial Δd MAE={result['lateral_mae_m']:.3f}m | max |Pred Δd|={result['max_abs_predicted_lateral_m']:.3f}m",
        "Speed is not learned. Runtime Velocity Planner + MPC owns the speed profile.",
        "Runtime requires >=80m route ahead; recommended Mission-Planner crop is 100m.",
    ]
    y = 835
    for line in lines:
        draw.text((815, y), line, font=_font(17), fill="#e2e8f0")
        y += 34
    path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(path, quality=94)


def _write_index(
    output_dir: Path,
    epoch: int,
    rows: list[dict[str, Any]],
    validation_state: dict[str, Any],
) -> None:
    groups: dict[str, list[str]] = {}
    state_errors: list[str] = []
    obstacle_groups: dict[str, list[str]] = {"static": [], "dynamic": []}
    for row in rows:
        outcome = str(row["motion_state_outcome"])
        outcome_class = outcome.lower()
        obstacle_kind = str(row.get("obstacle_kind", "none"))
        card = (
            f"<article class='card {outcome_class}'>"
            f"<a href='{html.escape(row['image'])}'><img loading='lazy' src='{html.escape(row['image'])}'></a>"
            f"<h3>#{row['dataset_sample_index']} {html.escape(row['name'])}</h3>"
            f"<p>blackout={str(row['gps_blackout']).lower()} · avoidance={str(row['avoidance']).lower()} · "
            f"<strong>{outcome}</strong> · {row['motion_state_target']}→{row['motion_state_prediction']} "
            f"({row['motion_state_confidence']:.3f})</p>"
            f"<p>Δd MAE {row['lateral_mae_m']:.3f}m · valid {row['valid_anchor_count']}/20 · "
            f"supervised to {row['farthest_valid_station_m']:.1f}m</p>"
            f"<p><a href='{html.escape(row['result_file'])}'>JSON</a></p></article>"
        )
        groups.setdefault(str(row["group"]), []).append(card)
        if outcome in {"FP", "FN"}:
            state_errors.append(card)
        if obstacle_kind in obstacle_groups:
            obstacle_groups[obstacle_kind].append(card)
    error_section = (
        "<section id='state-errors'><h2>STOP/DRIVE errors in fixed recheck set "
        f"({len(state_errors)})</h2><div class='grid'>{''.join(state_errors)}</div></section>"
        if state_errors
        else "<section id='state-errors'><h2>STOP/DRIVE errors in fixed recheck set (0)</h2></section>"
    )
    obstacle_sections = "".join(
        f"<section id='obstacle-{kind}'><h2>Verified {kind} obstacles ({len(cards)})</h2>"
        f"<div class='grid'>{''.join(cards)}</div></section>"
        for kind, cards in obstacle_groups.items()
    )
    sections = "".join(
        f"<section><h2>{html.escape(group)} ({len(cards)})</h2><div class='grid'>{''.join(cards)}</div></section>"
        for group, cards in groups.items()
    )
    if validation_state:
        stop_metrics = validation_state["states"]["STOP"]
        dashboard = f"""
<div class="metrics">
<div><b>Accuracy</b><span>{validation_state['accuracy'] * 100:.2f}%</span></div>
<div class="tp"><b>TP</b><span>{validation_state['tp']}</span></div>
<div class="fp"><b>FP</b><span>{validation_state['fp']} ({validation_state['false_positive_rate'] * 100:.2f}%)</span></div>
<div class="fn"><b>FN</b><span>{validation_state['fn']} ({validation_state['false_negative_rate'] * 100:.2f}%)</span></div>
<div><b>TN</b><span>{validation_state['tn']}</span></div>
<div><b>STOP precision</b><span>{stop_metrics['precision'] * 100:.2f}%</span></div>
<div><b>STOP recall</b><span>{stop_metrics['recall'] * 100:.2f}%</span></div>
<div><b>STOP F1</b><span>{stop_metrics['f1'] * 100:.2f}%</span></div>
</div>"""
    else:
        dashboard = "<p>Full-validation STOP/DRIVE metrics unavailable.</p>"
    document = f"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>V8 epoch {epoch} sample recheck</title><style>
body{{margin:0;padding:24px;background:#070b12;color:#e2e8f0;font:15px/1.45 system-ui,sans-serif}}
a{{color:#60a5fa}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(560px,1fr));gap:16px}}
.card{{background:#111827;border:1px solid #334155;border-radius:12px;padding:12px}}
.card.fp{{border-color:#f97316}}.card.fn{{border-color:#ef4444}}
.card img{{width:100%;border-radius:8px}}h2{{margin-top:36px}}
.nav a{{display:inline-block;margin:0 10px 8px 0;padding:8px 12px;background:#1e293b;border-radius:8px}}
.metrics{{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin:18px 0}}
.metrics div{{display:flex;flex-direction:column;background:#111827;border:1px solid #334155;border-radius:10px;padding:12px}}
.metrics span{{font-size:22px;margin-top:4px}}.metrics .fp{{border-color:#f97316}}.metrics .fn{{border-color:#ef4444}}
</style></head>
<body><h1>V8 epoch {epoch} fixed sample recheck</h1>
<p>{len(rows)} samples · spatial Δd only · learned speed disabled</p>
<nav class="nav"><a href="#state-errors">FP/FN</a><a href="#obstacle-static">Static obstacles</a>
<a href="#obstacle-dynamic">Dynamic obstacles</a></nav>
<h2>Full validation STOP/DRIVE (STOP = positive)</h2>{dashboard}
{error_section}{obstacle_sections}<section><h2>All configured groups</h2></section>{sections}</body></html>"""
    (output_dir / "index.html").write_text(document, encoding="utf-8")


def main() -> None:
    args = parse_args()
    source = json.loads(args.source_summary.read_text(encoding="utf-8"))
    requested = _merge_verified_obstacles(
        source.get("samples", []),
        args.verified_obstacle_manifest,
    )
    if args.max_samples > 0:
        requested = requested[: args.max_samples]
    device = torch.device("cpu" if args.cpu or not torch.cuda.is_available() else "cuda")
    checkpoint = torch.load(args.checkpoint, map_location=device, weights_only=False)
    epoch = int(checkpoint["epoch"]) + 1
    config_values = dict(checkpoint["model_config"])
    config_values["pretrained_camera"] = False
    model = SpatialResidualPlannerV8(ModelConfig(**config_values)).to(device)
    model.load_state_dict(checkpoint["model_state"])
    model.eval()
    splits = load_split_manifest(args.split_manifest)
    datasets: dict[str, PlannerDataset] = {}
    images = args.output_dir / "images"
    result_dir = args.output_dir / "results"
    images.mkdir(parents=True, exist_ok=True)
    result_dir.mkdir(parents=True, exist_ok=True)
    results: list[dict[str, Any]] = []
    train_args = checkpoint.get("train_args", {})
    for source_row in requested:
        split = str(source_row["split"])
        if split not in datasets:
            datasets[split] = PlannerDataset(
                args.data_root,
                splits[split],
                blackout_weight=1.0,
                stop_weight=1.0,
                drive_weight=1.0,
                avoidance_weight=1.0,
                avoidance_threshold_m=float(train_args.get("avoidance_threshold_m", 0.75)),
                allow_legacy_target_fields=args.allow_legacy_target_fields,
                photometric_augmentation=False,
            )
        sample_index = int(source_row["dataset_sample_index"])
        sample = datasets[split][sample_index]
        if sample["run_id"] != source_row["run_id"] or int(sample["sample_id"]) != int(source_row["sample_id"]):
            raise RuntimeError(f"sample identity changed at {split}[{sample_index}]")
        batch = make_batch(sample, device)
        with torch.inference_mode():
            outputs = model(
                batch["front"], batch["left"], batch["right"], batch["lidar_bev"],
                batch["ego"], batch["mgeo"], batch["local_route"],
            )
        valid = batch["target_spatial_valid"][0].bool()
        error = (
            outputs["lateral_residual_m"][0]
            - batch["target_spatial_lateral_m"][0]
        ).abs()
        valid_count = int(valid.sum())
        prediction_state = int(outputs["motion_state_prediction"][0])
        target_state = int(batch["motion_state"][0])
        name = str(source_row.get("name", f"sample_{sample_index}"))
        image_name = f"{sample_index:05d}_{_safe_name(name)}.jpg"
        json_name = f"{sample_index:05d}_{_safe_name(name)}.json"
        result = {
            "checkpoint": str(args.checkpoint.resolve()),
            "checkpoint_epoch": epoch,
            "split": split,
            "dataset_sample_index": sample_index,
            "run_id": str(sample["run_id"]),
            "sample_id": int(sample["sample_id"]),
            "gps_blackout": bool(batch["gps_blackout"][0]),
            "avoidance": bool(batch["avoidance"][0]),
            "name": name,
            "group": str(source_row.get("group", source_row.get("category", "representative"))),
            "motion_state_target": MOTION_STATE_NAMES[target_state],
            "motion_state_prediction": MOTION_STATE_NAMES[prediction_state],
            "motion_state_confidence": float(outputs["motion_state_probabilities"][0, prediction_state]),
            "motion_state_outcome": _state_outcome(
                MOTION_STATE_NAMES[target_state],
                MOTION_STATE_NAMES[prediction_state],
            ),
            "obstacle_kind": str(source_row.get("obstacle_kind", "none")),
            "obstacle_verification": str(
                source_row.get("obstacle_verification", "not_verified")
            ),
            "valid_anchor_count": valid_count,
            "farthest_valid_station_m": float(outputs["spatial_stations_m"][0, valid_count - 1]) if valid_count else 0.0,
            "lateral_mae_m": float(error[valid].mean()) if valid_count else 0.0,
            "max_abs_predicted_lateral_m": float(outputs["lateral_residual_m"][0].abs().max()),
            "image": f"images/{image_name}",
            "result_file": f"results/{json_name}",
        }
        _render(images / image_name, batch, outputs, result)
        (result_dir / json_name).write_text(json.dumps(result, indent=2), encoding="utf-8")
        results.append(result)
        print(json.dumps({"sample": sample_index, "name": name, "lateral_mae_m": result["lateral_mae_m"]}), flush=True)
    validation_state = _validation_state_summary(checkpoint)
    summary = {
        "checkpoint": str(args.checkpoint.resolve()),
        "checkpoint_epoch": epoch,
        "sample_count": len(results),
        "validation_motion_state": validation_state,
        "verified_obstacle_counts": {
            kind: sum(row["obstacle_kind"] == kind for row in results)
            for kind in ("static", "dynamic")
        },
        "samples": results,
    }
    (args.output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    _write_index(args.output_dir, epoch, results, validation_state)
    print(json.dumps({"complete": True, "gallery": str(args.output_dir / "index.html")}), flush=True)


if __name__ == "__main__":
    main()
