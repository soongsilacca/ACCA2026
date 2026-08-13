from __future__ import annotations

import heapq
import html
import json
import math
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch
from PIL import Image, ImageDraw, ImageOps

from multimodal_planner_v5.metrics import trajectory_diagnostics


IMAGENET_MEAN = np.asarray([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.asarray([0.229, 0.224, 0.225], dtype=np.float32)
PREDICTION_COLOR = (239, 83, 80)


def _image_from_normalized_tensor(value: torch.Tensor) -> np.ndarray:
    image = value.detach().float().cpu().permute(1, 2, 0).numpy()
    image = (image * IMAGENET_STD + IMAGENET_MEAN) * 255.0
    return np.clip(image, 0.0, 255.0).astype(np.uint8)


@dataclass
class OutlierSample:
    score: float
    run_id: str
    sample_id: int
    gps_blackout: bool
    ade_m: float
    fde_m: float
    yaw_mae_deg: float
    step_mae_m: float
    acceleration_mae_m: float
    heading_consistency_mae_deg: float
    first_waypoint_error_m: float
    predicted_first_x_m: float
    target_first_x_m: float
    origin_direction_mismatch: float
    gt_relative_lateral_sign_flip_count: float
    gt_relative_lateral_sign_flip_rate: float
    front: np.ndarray
    left: np.ndarray
    right: np.ndarray
    target: np.ndarray
    prediction: np.ndarray

    def metadata(self) -> dict[str, Any]:
        result = asdict(self)
        for key in ("front", "left", "right", "target", "prediction"):
            result.pop(key)
        return result


class TopOutlierCollector:
    """Keep the largest single-trajectory ADE examples."""

    def __init__(self, count: int) -> None:
        if count <= 0:
            raise ValueError("outlier count must be positive")
        self.count = count
        self._sequence = 0
        self._overall: list[tuple[float, int, OutlierSample]] = []
        self._blackout: list[tuple[float, int, OutlierSample]] = []

    def _would_accept(
        self, heap: list[tuple[float, int, OutlierSample]], score: float
    ) -> bool:
        return len(heap) < self.count or score > heap[0][0]

    def _push(
        self,
        heap: list[tuple[float, int, OutlierSample]],
        sample: OutlierSample,
    ) -> None:
        item = (sample.score, self._sequence, sample)
        self._sequence += 1
        if len(heap) < self.count:
            heapq.heappush(heap, item)
        else:
            heapq.heapreplace(heap, item)

    def update(
        self,
        outputs: dict[str, torch.Tensor],
        batch: dict[str, Any],
        diagnostics: dict[str, torch.Tensor],
    ) -> None:
        for index in range(outputs["trajectory"].shape[0]):
            score = float(diagnostics["ade_m"][index])
            blackout = bool(batch["gps_blackout"][index])
            accept_overall = self._would_accept(self._overall, score)
            accept_blackout = blackout and self._would_accept(self._blackout, score)
            if not accept_overall and not accept_blackout:
                continue
            sample = OutlierSample(
                score=score,
                run_id=str(batch["run_id"][index]),
                sample_id=int(batch["sample_id"][index]),
                gps_blackout=blackout,
                ade_m=score,
                fde_m=float(diagnostics["fde_m"][index]),
                yaw_mae_deg=float(diagnostics["yaw_mae_deg"][index]),
                step_mae_m=float(diagnostics["step_mae_m"][index]),
                acceleration_mae_m=float(
                    diagnostics["acceleration_mae_m"][index]
                ),
                heading_consistency_mae_deg=float(
                    diagnostics["heading_consistency_mae_deg"][index]
                ),
                first_waypoint_error_m=float(
                    diagnostics["first_waypoint_error_m"][index]
                ),
                predicted_first_x_m=float(
                    diagnostics["predicted_first_x_m"][index]
                ),
                target_first_x_m=float(
                    diagnostics["target_first_x_m"][index]
                ),
                origin_direction_mismatch=float(
                    diagnostics["origin_direction_mismatch"][index]
                ),
                gt_relative_lateral_sign_flip_count=float(
                    diagnostics["gt_relative_lateral_sign_flip_count"][index]
                ),
                gt_relative_lateral_sign_flip_rate=float(
                    diagnostics["gt_relative_lateral_sign_flip_rate"][index]
                ),
                front=_image_from_normalized_tensor(batch["front"][index, -1]),
                left=_image_from_normalized_tensor(batch["left"][index, -1]),
                right=_image_from_normalized_tensor(batch["right"][index, -1]),
                target=batch["target"][index].detach().float().cpu().numpy(),
                prediction=outputs["trajectory"][index]
                .detach()
                .float()
                .cpu()
                .numpy(),
            )
            if accept_overall:
                self._push(self._overall, sample)
            if accept_blackout:
                self._push(self._blackout, sample)

    @staticmethod
    def _sorted(
        heap: list[tuple[float, int, OutlierSample]],
    ) -> list[OutlierSample]:
        return [item[2] for item in sorted(heap, reverse=True)]

    def export(self, output_root: Path, epoch: int) -> dict[str, Any]:
        epoch_dir = output_root / f"epoch_{epoch:03d}"
        epoch_dir.mkdir(parents=True, exist_ok=True)
        groups = {
            "overall": self._sorted(self._overall),
            "blackout": self._sorted(self._blackout),
        }
        manifest: dict[str, Any] = {
            "epoch": epoch,
            "ranking_metric": "ade_m",
            "groups": {},
        }
        for group_name, samples in groups.items():
            rows = []
            for rank, sample in enumerate(samples, start=1):
                safe_run = re.sub(r"[^A-Za-z0-9_.-]+", "_", sample.run_id)
                filename = (
                    f"{group_name}_{rank:03d}_{safe_run}_"
                    f"sample_{sample.sample_id}.png"
                )
                _render_sample(sample, epoch_dir / filename)
                row = sample.metadata()
                row["rank"] = rank
                row["image"] = filename
                rows.append(row)
            manifest["groups"][group_name] = rows
        (epoch_dir / "manifest.json").write_text(
            json.dumps(manifest, indent=2), encoding="utf-8"
        )
        (epoch_dir / "index.html").write_text(
            _gallery_html(epoch, manifest), encoding="utf-8"
        )
        _write_root_index(output_root)
        return {
            "directory": str(epoch_dir),
            "gallery": str(epoch_dir / "index.html"),
            "manifest": str(epoch_dir / "manifest.json"),
            "overall_count": len(groups["overall"]),
            "blackout_count": len(groups["blackout"]),
            "ranking_metric": "ade_m",
        }


def _fit_image(image: np.ndarray, size: tuple[int, int]) -> Image.Image:
    return ImageOps.fit(
        Image.fromarray(image), size, method=Image.Resampling.BILINEAR
    )


def _render_sample(sample: OutlierSample, path: Path) -> None:
    canvas = Image.new("RGB", (1400, 900), (20, 23, 29))
    draw = ImageDraw.Draw(canvas)
    panels = (
        ("LEFT", sample.left, (20, 55), (320, 240)),
        ("FRONT", sample.front, (360, 25), (640, 360)),
        ("RIGHT", sample.right, (1020, 55), (320, 240)),
    )
    for label, value, position, size in panels:
        canvas.paste(_fit_image(value, size), position)
        draw.text((position[0], position[1] - 20), label, fill=(230, 232, 236))
    draw.text(
        (20, 8),
        f"V5 K=1 | run={sample.run_id} | sample={sample.sample_id} | "
        f"GPS blackout={sample.gps_blackout}",
        fill=(246, 247, 249),
    )
    _draw_trajectory_plot(draw, (40, 420, 760, 850), sample)
    lines = [
        "Ranking: ADE (single trajectory)",
        f"ADE: {sample.ade_m:.3f} m",
        f"FDE: {sample.fde_m:.3f} m",
        f"Yaw MAE: {sample.yaw_mae_deg:.3f} deg",
        f"Step MAE: {sample.step_mae_m:.3f} m",
        f"Acceleration MAE: {sample.acceleration_mae_m:.3f} m",
        f"XY-heading consistency MAE: "
        f"{sample.heading_consistency_mae_deg:.3f} deg",
        f"First waypoint error: {sample.first_waypoint_error_m:.3f} m",
        f"First forward x pred/GT: {sample.predicted_first_x_m:.3f} / "
        f"{sample.target_first_x_m:.3f} m",
        f"Origin direction mismatch: "
        f"{bool(sample.origin_direction_mismatch)}",
        f"GT-relative lateral sign flips: "
        f"{sample.gt_relative_lateral_sign_flip_count:.0f}",
        f"GT-relative lateral flip rate: "
        f"{sample.gt_relative_lateral_sign_flip_rate:.3f}",
        "",
        "White: ground truth",
        "Red: K=1 prediction",
    ]
    text_y = 430
    for line in lines:
        draw.text((810, text_y), line, fill=(225, 228, 234))
        text_y += 29
    path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(path, format="PNG", optimize=True)


def _draw_trajectory_plot(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    sample: OutlierSample,
) -> None:
    left, top, right, bottom = box
    draw.rectangle(box, outline=(100, 108, 120), width=2)
    origin = np.zeros((1, 2), dtype=np.float32)
    target_xy = np.concatenate((origin, sample.target[:, :2] * 50.0), axis=0)
    prediction_xy = np.concatenate(
        (origin, sample.prediction[:, :2] * 50.0), axis=0
    )
    all_xy = np.concatenate((target_xy, prediction_xy), axis=0)
    forward_min = min(-5.0, float(np.nanmin(all_xy[:, 0])) - 2.0)
    forward_max = max(30.0, float(np.nanmax(all_xy[:, 0])) + 2.0)
    lateral_extent = max(12.0, float(np.nanmax(np.abs(all_xy[:, 1]))) + 2.0)
    usable_w = right - left - 70
    usable_h = bottom - top - 50
    center_x = (left + right) / 2.0

    def project(point: np.ndarray) -> tuple[int, int]:
        x = center_x - float(point[1]) / lateral_extent * (usable_w / 2.0)
        y = bottom - 25 - (
            (float(point[0]) - forward_min)
            / (forward_max - forward_min)
            * usable_h
        )
        return int(round(x)), int(round(y))

    value = math.ceil(forward_min / 10.0) * 10.0
    while value <= forward_max:
        y = project(np.asarray([value, 0.0]))[1]
        draw.line((left + 20, y, right - 20, y), fill=(48, 54, 64), width=1)
        draw.text((left + 4, y - 7), f"{value:.0f}m", fill=(145, 151, 162))
        value += 10.0
    center_line = project(np.asarray([0.0, 0.0]))[0]
    draw.line(
        (center_line, top + 15, center_line, bottom - 20),
        fill=(65, 71, 82),
        width=1,
    )
    predicted = [project(point) for point in prediction_xy]
    draw.line(predicted, fill=PREDICTION_COLOR, width=6, joint="curve")
    for point in predicted[::4]:
        draw.ellipse(
            (point[0] - 3, point[1] - 3, point[0] + 3, point[1] + 3),
            fill=PREDICTION_COLOR,
        )
    expected = [project(point) for point in target_xy]
    draw.line(expected, fill=(250, 250, 250), width=5, joint="curve")
    ego = project(np.asarray([0.0, 0.0]))
    draw.polygon(
        (
            (ego[0], ego[1] - 10),
            (ego[0] - 8, ego[1] + 8),
            (ego[0] + 8, ego[1] + 8),
        ),
        fill=(255, 209, 89),
    )


def _gallery_html(epoch: int, manifest: dict[str, Any]) -> str:
    sections = []
    for group_name, rows in manifest["groups"].items():
        cards = []
        for row in rows:
            cards.append(
                """
                <article class="card">
                  <a href="{image}"><img loading="lazy" src="{image}"></a>
                  <div class="meta">
                    <strong>#{rank} · ADE {ade:.3f}m · FDE {fde:.3f}m</strong><br>
                    {run_id} · sample {sample_id} · blackout={blackout}
                  </div>
                </article>
                """.format(
                    image=html.escape(row["image"]),
                    rank=row["rank"],
                    ade=row["ade_m"],
                    fde=row["fde_m"],
                    run_id=html.escape(row["run_id"]),
                    sample_id=row["sample_id"],
                    blackout=str(row["gps_blackout"]).lower(),
                )
            )
        sections.append(
            f"<h2>{html.escape(group_name)} ({len(rows)})</h2>"
            f"<section class=\"grid\">{''.join(cards)}</section>"
        )
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>V5 validation outliers · epoch {epoch}</title>
  <style>
    body {{ margin: 0; padding: 28px; background: #11151b; color: #edf0f5;
            font-family: system-ui, sans-serif; }}
    a {{ color: #8fc7ff; }}
    .grid {{ display: grid; grid-template-columns: repeat(auto-fit,minmax(420px,1fr));
             gap: 18px; }}
    .card {{ background: #1c222c; border: 1px solid #333c49; border-radius: 10px;
             overflow: hidden; }}
    .card img {{ display: block; width: 100%; height: auto; }}
    .meta {{ padding: 12px; line-height: 1.5; overflow-wrap: anywhere; }}
  </style>
</head>
<body>
  <h1>V5 K=1 validation outliers · epoch {epoch}</h1>
  <p>Ranked by ADE of the single predicted trajectory.</p>
  {''.join(sections)}
</body>
</html>
"""


def _write_root_index(output_root: Path) -> None:
    epoch_dirs = sorted(
        path
        for path in output_root.glob("epoch_*")
        if path.is_dir() and (path / "index.html").is_file()
    )
    links = "\n".join(
        f'<li><a href="{path.name}/index.html">{path.name}</a></li>'
        for path in reversed(epoch_dirs)
    )
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "index.html").write_text(
        f"""<!doctype html>
<html lang="en">
<head><meta charset="utf-8"><title>V5 validation outlier epochs</title></head>
<body style="font-family:system-ui,sans-serif;max-width:800px;margin:40px auto">
<h1>V5 validation outlier galleries</h1>
<ul>{links}</ul>
</body>
</html>
""",
        encoding="utf-8",
    )
