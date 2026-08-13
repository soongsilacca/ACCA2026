from __future__ import annotations

import argparse
import html
import json
import re
from pathlib import Path
from typing import Any

import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont

from multimodal_planner_v5.data import IMAGENET_MEAN, IMAGENET_STD
from multimodal_planner_v5.visualize_sample import make_batch
from multimodal_planner_v7.data import MOTION_STATE_NAMES, PlannerDataset, load_split_manifest
from multimodal_planner_v7.metrics import trajectory_diagnostics
from multimodal_planner_v7.model import (
    SPEED_SCALE_MPS,
    XY_SCALE_M,
    ModelConfig,
    RouteLockedSpeedPlannerV7,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render a fixed representative sample set with a V7 checkpoint."
    )
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--source-summary", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--split-manifest",
        type=Path,
        default=Path("multimodal_planner_v5/splits/v001.json"),
    )
    parser.add_argument("--allow-legacy-target-fields", action="store_true")
    parser.add_argument("--max-samples", type=int, default=0)
    parser.add_argument("--cpu", action="store_true")
    return parser.parse_args()


def _font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
    ):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            pass
    return ImageFont.load_default()


def _camera_image(tensor: torch.Tensor) -> Image.Image:
    value = tensor.detach().float().cpu().numpy()
    if value.ndim == 4:
        value = value[-1]
    value = np.transpose(value, (1, 2, 0))
    value = np.clip((value * IMAGENET_STD + IMAGENET_MEAN) * 255.0, 0, 255)
    return Image.fromarray(value.astype(np.uint8), mode="RGB")


def _fit(image: Image.Image, size: tuple[int, int]) -> Image.Image:
    copy = image.copy()
    copy.thumbnail(size, Image.Resampling.LANCZOS)
    result = Image.new("RGB", size, "#090d14")
    result.paste(copy, ((size[0] - copy.width) // 2, (size[1] - copy.height) // 2))
    return result


def _panel(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    title: str,
) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = box
    draw.rounded_rectangle(box, radius=10, fill="#111827", outline="#334155", width=2)
    draw.text((x0 + 14, y0 + 10), title, font=_font(20), fill="#f8fafc")
    return x0 + 48, y0 + 52, x1 - 24, y1 - 38


def _plot_xy(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    route: np.ndarray,
    target: np.ndarray,
    prediction: np.ndarray,
) -> None:
    x0, y0, x1, y1 = _panel(draw, box, "Local Route / GT / Prediction")
    all_points = np.concatenate((route, target, prediction, np.zeros((1, 2))), axis=0)
    xmin, ymin = np.min(all_points, axis=0)
    xmax, ymax = np.max(all_points, axis=0)
    margin = max(float(xmax - xmin), float(ymax - ymin), 5.0) * 0.08
    xmin, xmax = float(xmin - margin), float(xmax + margin)
    ymin, ymax = float(ymin - margin), float(ymax + margin)
    scale = min((x1 - x0) / max(xmax - xmin, 1e-4), (y1 - y0) / max(ymax - ymin, 1e-4))
    cx = (x0 + x1) / 2 - scale * (xmin + xmax) / 2
    cy = (y0 + y1) / 2 + scale * (ymin + ymax) / 2

    def points(values: np.ndarray) -> list[tuple[float, float]]:
        return [(cx + scale * float(x), cy - scale * float(y)) for x, y in values]

    origin = points(np.zeros((1, 2)))[0]
    draw.line((x0, origin[1], x1, origin[1]), fill="#263244", width=1)
    draw.line((origin[0], y0, origin[0], y1), fill="#263244", width=1)
    draw.line(points(route), fill="#94a3b8", width=4)
    draw.line(points(target), fill="#22c55e", width=6)
    draw.line(points(prediction), fill="#ef4444", width=5)
    for point in points(target):
        draw.ellipse((point[0] - 3, point[1] - 3, point[0] + 3, point[1] + 3), fill="#22c55e")
    for point in points(prediction):
        draw.ellipse((point[0] - 3, point[1] - 3, point[0] + 3, point[1] + 3), fill="#ef4444")
    draw.ellipse((origin[0] - 6, origin[1] - 6, origin[0] + 6, origin[1] + 6), fill="#f8fafc")
    legend_y = y1 + 9
    for offset, color, label in (
        (0, "#94a3b8", "Local Route"),
        (180, "#22c55e", "GT"),
        (270, "#ef4444", "Pred"),
    ):
        draw.line((x0 + offset, legend_y, x0 + offset + 32, legend_y), fill=color, width=5)
        draw.text((x0 + offset + 40, legend_y - 9), label, font=_font(15), fill="#e2e8f0")


def _plot_series(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    title: str,
    series: list[tuple[str, str, np.ndarray]],
    unit: str,
) -> None:
    x0, y0, x1, y1 = _panel(draw, box, title)
    values = np.concatenate([value.reshape(-1) for _, _, value in series])
    ymin, ymax = float(values.min()), float(values.max())
    pad = max((ymax - ymin) * 0.12, 0.25)
    ymin, ymax = ymin - pad, ymax + pad
    draw.line((x0, y1, x1, y1), fill="#475569", width=1)
    draw.line((x0, y0, x0, y1), fill="#475569", width=1)
    if ymin <= 0.0 <= ymax:
        zero_y = y1 - (0.0 - ymin) / (ymax - ymin) * (y1 - y0)
        draw.line((x0, zero_y, x1, zero_y), fill="#334155", width=1)
    for name, color, value in series:
        xs = np.linspace(x0, x1, len(value))
        ys = y1 - (value - ymin) / (ymax - ymin) * (y1 - y0)
        draw.line(list(zip(xs.tolist(), ys.tolist())), fill=color, width=4)
    draw.text((x0, y0 - 30), f"{ymax:.2f}", font=_font(13), fill="#94a3b8")
    draw.text((x0, y1 + 8), f"{ymin:.2f} {unit}", font=_font(13), fill="#94a3b8")
    legend_x = x0 + 95
    for name, color, _ in series:
        draw.line((legend_x, y1 + 18, legend_x + 28, y1 + 18), fill=color, width=4)
        draw.text((legend_x + 34, y1 + 8), name, font=_font(14), fill="#e2e8f0")
        legend_x += 120


def _safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "sample"


def _sample_metrics(diagnostics: dict[str, torch.Tensor]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for name in (
        "ade_m",
        "fde_m",
        "yaw_mae_deg",
        "longitudinal_mae_m",
        "lateral_mae_m",
    ):
        result[name] = float(diagnostics[name][0])
    result["horizon_metrics"] = {
        f"{seconds}s": {
            "ade_m": float(diagnostics[f"ade_{seconds}s_m"][0]),
            "fde_m": float(diagnostics[f"fde_{seconds}s_m"][0]),
        }
        for seconds in (1, 2, 4)
    }
    return result


def _render(
    path: Path,
    batch: dict[str, Any],
    outputs: dict[str, torch.Tensor],
    result: dict[str, Any],
) -> None:
    canvas = Image.new("RGB", (1600, 1240), "#070b12")
    draw = ImageDraw.Draw(canvas)
    front = _fit(_camera_image(batch["front"][0]), (760, 340))
    left = _fit(_camera_image(batch["left"][0]), (380, 280))
    right = _fit(_camera_image(batch["right"][0]), (380, 280))
    canvas.paste(front, (20, 42))
    canvas.paste(left, (800, 42))
    canvas.paste(right, (1200, 42))
    draw.text((20, 12), "Front", font=_font(20), fill="#f8fafc")
    draw.text((800, 12), "Left", font=_font(20), fill="#f8fafc")
    draw.text((1200, 12), "Right", font=_font(20), fill="#f8fafc")

    route = batch["local_route"][0, :, :2].detach().float().cpu().numpy() * XY_SCALE_M
    target = batch["target"][0, :, :2].detach().float().cpu().numpy() * XY_SCALE_M
    prediction = outputs["trajectory"][0, :, :2].detach().float().cpu().numpy() * XY_SCALE_M
    _plot_xy(draw, (20, 405, 780, 1210), route, target, prediction)

    target_lateral = batch["target_lateral_residual_m"][0].detach().float().cpu().numpy()
    predicted_lateral = outputs["lateral_residual_m"][0].detach().float().cpu().numpy()
    _plot_series(
        draw,
        (800, 405, 1580, 710),
        "Path-normal residual Δd (GT follows avoidance label)",
        [("GT Δd", "#22c55e", target_lateral), ("Pred Δd", "#ef4444", predicted_lateral)],
        "m",
    )

    target_speed = batch["target"][0, :, 3].detach().float().cpu().numpy() * SPEED_SCALE_MPS
    base_speed = outputs["base_speed"][0].detach().float().cpu().numpy() * SPEED_SCALE_MPS
    speed_delta = outputs["speed_delta"][0].detach().float().cpu().numpy() * SPEED_SCALE_MPS
    predicted_speed = outputs["future_speed"][0].detach().float().cpu().numpy() * SPEED_SCALE_MPS
    _plot_series(
        draw,
        (800, 735, 1580, 1040),
        "Speed: fixed MGeo base + learned Δspeed",
        [
            ("Target", "#22c55e", target_speed),
            ("Base", "#94a3b8", base_speed),
            ("Δspeed", "#f59e0b", speed_delta),
            ("Final", "#ef4444", predicted_speed),
        ],
        "m/s",
    )
    lines = [
        f"#{result['dataset_sample_index']}  {result['name']}  |  {result['group']}",
        f"{result['run_id']} / sample {result['sample_id']}  |  GPS blackout={result['gps_blackout']}  |  avoidance={result['avoidance']}",
        f"state GT={result['motion_state_target']}  Pred={result['motion_state_prediction']} ({result['motion_state_confidence']:.3f})",
        f"ADE/FDE: 1s {result['horizon_metrics']['1s']['ade_m']:.2f}/{result['horizon_metrics']['1s']['fde_m']:.2f} m"
        f"  |  2s {result['horizon_metrics']['2s']['ade_m']:.2f}/{result['horizon_metrics']['2s']['fde_m']:.2f} m"
        f"  |  4s {result['horizon_metrics']['4s']['ade_m']:.2f}/{result['horizon_metrics']['4s']['fde_m']:.2f} m",
        f"4s route progress GT/Pred={result['target_progress_4s_m']:.2f}/{result['predicted_progress_4s_m']:.2f} m"
        f"  |  trajectory horizon=20 points × 0.2 s",
        f"lateral residual MAE={result['route_residual_mae_m']:.2f} m  |  speed MAE={result['speed_mae_mps']:.2f} m/s"
        f"  |  max |Δd| GT/Pred={result['target_max_abs_lateral_residual_m']:.2f}/{result['predicted_max_abs_lateral_residual_m']:.2f} m",
    ]
    y = 1060
    for line in lines:
        draw.text((815, y), line, font=_font(16), fill="#e2e8f0")
        y += 30
    path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(path, quality=94)


def _write_index(output_dir: Path, epoch: int, rows: list[dict[str, Any]]) -> None:
    groups: dict[str, list[str]] = {}
    for row in rows:
        horizon = row["horizon_metrics"]
        card = (
            "<article class='card'>"
            f"<a href='{html.escape(row['image'])}'><img loading='lazy' src='{html.escape(row['image'])}'></a>"
            f"<h3>#{row['dataset_sample_index']} {html.escape(row['name'])}</h3>"
            f"<p>{html.escape(row['run_id'])} / {row['sample_id']} · blackout={str(row['gps_blackout']).lower()} · "
            f"avoidance={str(row['avoidance']).lower()}</p>"
            f"<p>state {row['motion_state_target']} → {row['motion_state_prediction']} ({row['motion_state_confidence']:.3f})</p>"
            f"<p>ADE/FDE 1s {horizon['1s']['ade_m']:.2f}/{horizon['1s']['fde_m']:.2f}m · "
            f"2s {horizon['2s']['ade_m']:.2f}/{horizon['2s']['fde_m']:.2f}m · "
            f"4s {horizon['4s']['ade_m']:.2f}/{horizon['4s']['fde_m']:.2f}m</p>"
            f"<p>4s route progress GT/Pred {row['target_progress_4s_m']:.2f}/"
            f"{row['predicted_progress_4s_m']:.2f}m</p>"
            f"<p>Δd MAE {row['route_residual_mae_m']:.2f}m · "
            f"max |Δd| GT/Pred {row['target_max_abs_lateral_residual_m']:.2f}/"
            f"{row['predicted_max_abs_lateral_residual_m']:.2f}m · "
            f"speed MAE {row['speed_mae_mps']:.2f}m/s</p>"
            f"<p><a href='{html.escape(row['result_file'])}'>JSON</a></p></article>"
        )
        groups.setdefault(str(row["group"]), []).append(card)
    sections = "".join(
        f"<section><h2>{html.escape(group)} ({len(cards)})</h2><div class='grid'>{''.join(cards)}</div></section>"
        for group, cards in groups.items()
    )
    document = f"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>V7 epoch {epoch} sample recheck</title><style>
body{{margin:0;padding:24px;background:#070b12;color:#e2e8f0;font:15px/1.45 system-ui,sans-serif}}
a{{color:#60a5fa}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(560px,1fr));gap:16px}}
.card{{background:#111827;border:1px solid #334155;border-radius:12px;padding:12px}}
.card img{{width:100%;border-radius:8px}}h2{{margin-top:36px}}</style></head>
<body><h1>V7 epoch {epoch} fixed sample recheck</h1>
<p>{len(rows)} samples · Local Route / GT / Pred · Δd · base/Δ/final speed</p>{sections}</body></html>"""
    (output_dir / "index.html").write_text(document, encoding="utf-8")


def main() -> None:
    args = parse_args()
    source = json.loads(args.source_summary.read_text(encoding="utf-8"))
    requested = source.get("samples", [])
    if args.max_samples > 0:
        requested = requested[: args.max_samples]
    if not requested:
        raise ValueError(f"{args.source_summary}: no samples")

    device = torch.device("cpu" if args.cpu or not torch.cuda.is_available() else "cuda")
    checkpoint = torch.load(args.checkpoint, map_location=device, weights_only=False)
    train_args = checkpoint.get("train_args", {})
    epoch = int(checkpoint["epoch"]) + 1
    model = RouteLockedSpeedPlannerV7(
        ModelConfig(**checkpoint["model_config"]),
        normal_base_speed_mps=float(train_args.get("normal_base_speed_mps", 59.0 / 3.6)),
        speed_zone_base_speed_mps=float(train_args.get("speed_zone_base_speed_mps", 20.0)),
    ).to(device)
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    splits = load_split_manifest(args.split_manifest)
    datasets: dict[str, PlannerDataset] = {}
    results: list[dict[str, Any]] = []
    images_dir = args.output_dir / "images"
    results_dir = args.output_dir / "results"
    images_dir.mkdir(parents=True, exist_ok=True)
    results_dir.mkdir(parents=True, exist_ok=True)

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
                stop_speed_threshold_mps=float(train_args.get("stop_speed_threshold", 0.6)),
                stop_max_endpoint_distance_m=float(train_args.get("stop_max_endpoint_distance", 10.0)),
            )
        dataset = datasets[split]
        sample_index = int(source_row["dataset_sample_index"])
        sample = dataset[sample_index]
        if sample["run_id"] != source_row["run_id"] or int(sample["sample_id"]) != int(source_row["sample_id"]):
            raise RuntimeError(
                f"sample identity changed at {split}[{sample_index}]: "
                f"{sample['run_id']}/{sample['sample_id']}"
            )
        batch = make_batch(sample, device)
        with torch.inference_mode(), torch.autocast(
            device_type=device.type, dtype=torch.float16, enabled=device.type == "cuda"
        ):
            outputs = model(
                batch["front"], batch["left"], batch["right"], batch["lidar_bev"],
                batch["ego"], batch["mgeo"], batch["local_route"],
            )
        diagnostics = trajectory_diagnostics(outputs, batch["target"])
        target_state = int(batch["motion_state"][0].detach().cpu())
        predicted_state = int(outputs["motion_state_prediction"][0].detach().cpu())
        confidence = float(outputs["motion_state_probabilities"][0, predicted_state].detach().cpu())
        speed_error = (
            outputs["future_speed"][0].detach().float()
            - batch["target"][0, :, 3].detach().float()
        ) * SPEED_SCALE_MPS
        lateral_error = (
            outputs["lateral_residual_m"][0].detach().float()
            - batch["target_lateral_residual_m"][0].detach().float()
        )
        name = str(source_row.get("name", f"sample_{sample_index}"))
        image_name = f"{sample_index:05d}_{_safe_name(name)}.jpg"
        result_name = f"{sample_index:05d}_{_safe_name(name)}.json"
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
            "category": str(source_row.get("category", "representative")),
            "group": str(source_row.get("group", source_row.get("category", "representative"))),
            "motion_state_target": MOTION_STATE_NAMES[target_state],
            "motion_state_prediction": MOTION_STATE_NAMES[predicted_state],
            "motion_state_confidence": confidence,
            "speed_mae_mps": float(speed_error.abs().mean().cpu()),
            "route_residual_mae_m": float(lateral_error.abs().mean().cpu()),
            "target_progress_4s_m": float(batch["target_route_progress_m"][0, -1].detach().cpu()),
            "predicted_progress_4s_m": float(outputs["forward_progress_m"][0, -1].detach().cpu()),
            "target_max_abs_lateral_residual_m": float(batch["target_lateral_residual_m"][0].abs().max().detach().cpu()),
            "predicted_max_abs_lateral_residual_m": float(outputs["lateral_residual_m"][0].abs().max().detach().cpu()),
            "target_final_lateral_residual_m": float(batch["target_lateral_residual_m"][0, -1].detach().cpu()),
            "predicted_final_lateral_residual_m": float(outputs["lateral_residual_m"][0, -1].detach().cpu()),
            "image": f"images/{image_name}",
            "result_file": f"results/{result_name}",
            **_sample_metrics(diagnostics),
        }
        _render(images_dir / image_name, batch, outputs, result)
        (results_dir / result_name).write_text(json.dumps(result, indent=2), encoding="utf-8")
        results.append(result)
        print(json.dumps({
            "sample": sample_index,
            "name": name,
            "ade_m": result["ade_m"],
            "speed_mae_mps": result["speed_mae_mps"],
            "route_residual_mae_m": result["route_residual_mae_m"],
        }, sort_keys=True), flush=True)

    summary = {
        "checkpoint": str(args.checkpoint.resolve()),
        "checkpoint_epoch": epoch,
        "sample_count": len(results),
        "device": str(device),
        "samples": results,
    }
    (args.output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    _write_index(args.output_dir, epoch, results)
    print(json.dumps({"complete": True, "sample_count": len(results), "gallery": str(args.output_dir / "index.html")}), flush=True)


if __name__ == "__main__":
    main()
