from __future__ import annotations

import argparse
import html
import json
from pathlib import Path
from typing import Any

import torch

from multimodal_planner_v5.data import PlannerDataset, load_split_manifest
from multimodal_planner_v5.metrics import trajectory_diagnostics
from multimodal_planner_v5.model import (
    ModelConfig,
    MultiViewTemporalTrajectoryPlannerV5,
)
from multimodal_planner_v5.outliers import TopOutlierCollector
from multimodal_planner_v5.pack_recheck_gallery import write_flat_slideshow
from multimodal_planner_v5.visualize_sample import make_batch


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Re-run a saved representative-sample set with a V5 checkpoint."
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
    parser.add_argument(
        "--max-samples",
        type=int,
        default=0,
        help="limit the saved sample list for a smoke test; 0 runs all",
    )
    parser.add_argument("--cpu", action="store_true")
    return parser.parse_args()


def _sample_metrics(
    diagnostics: dict[str, torch.Tensor],
) -> dict[str, Any]:
    return {
        "ade_m": float(diagnostics["ade_m"][0]),
        "fde_m": float(diagnostics["fde_m"][0]),
        "yaw_mae_deg": float(diagnostics["yaw_mae_deg"][0]),
        "longitudinal_mae_m": float(diagnostics["longitudinal_mae_m"][0]),
        "lateral_mae_m": float(diagnostics["lateral_mae_m"][0]),
        "step_mae_m": float(diagnostics["step_mae_m"][0]),
        "acceleration_mae_m": float(
            diagnostics["acceleration_mae_m"][0]
        ),
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
    }


def _write_index(output_dir: Path, epoch: int, rows: list[dict[str, Any]]) -> None:
    table_rows = []
    grouped_cards: dict[str, list[str]] = {}
    for row in rows:
        horizons = row["horizon_metrics"]
        group = str(row.get("group", row["category"]))
        similarity_rank = int(row.get("similarity_rank", 0))
        similarity = row.get("similarity", {})
        similarity_score = float(similarity.get("score", 0.0))
        table_rows.append(
            "<tr>"
            f"<td>{row['dataset_sample_index']}</td>"
            f"<td>{html.escape(row['name'])}</td>"
            f"<td>{html.escape(row['category'])}</td>"
            f"<td>{html.escape(group)}</td>"
            f"<td>{similarity_rank}</td>"
            f"<td>{similarity_score:.3f}</td>"
            f"<td>{horizons['1s']['ade_m']:.2f} / "
            f"{horizons['1s']['fde_m']:.2f}</td>"
            f"<td>{horizons['2s']['ade_m']:.2f} / "
            f"{horizons['2s']['fde_m']:.2f}</td>"
            f"<td>{horizons['4s']['ade_m']:.2f} / "
            f"{horizons['4s']['fde_m']:.2f}</td>"
            f"<td>{row['lateral_mae_m']:.2f}</td>"
            f"<td>{row['longitudinal_mae_m']:.2f}</td>"
            f"<td>{row['yaw_mae_deg']:.2f}</td>"
            f"<td>{row['predicted_first_x_m']:.2f} / "
            f"{row['target_first_x_m']:.2f}</td>"
            f"<td>{str(row['origin_direction_mismatch']).lower()}</td>"
            f"<td>{row['gt_relative_lateral_sign_flip_count']:.0f}</td>"
            "</tr>"
        )
        grouped_cards.setdefault(group, []).append(
            "<article class='card'>"
            f"<a href='{html.escape(row['image'])}'>"
            f"<img loading='lazy' src='{html.escape(row['image'])}'></a>"
            f"<h3>#{row['dataset_sample_index']} {html.escape(row['name'])}</h3>"
            f"<p>{html.escape(row['category'])} · "
            f"rank={similarity_rank} · similarity={similarity_score:.3f} · "
            f"blackout={str(row['gps_blackout']).lower()}</p>"
            f"<p>ADE/FDE @1s {horizons['1s']['ade_m']:.2f}/"
            f"{horizons['1s']['fde_m']:.2f}m · "
            f"@2s {horizons['2s']['ade_m']:.2f}/"
            f"{horizons['2s']['fde_m']:.2f}m · "
            f"@4s {horizons['4s']['ade_m']:.2f}/"
            f"{horizons['4s']['fde_m']:.2f}m</p>"
            f"<p><a href='{html.escape(row['result_file'])}'>result JSON</a> · "
            f"<a href='{html.escape(row['gallery'])}'>detail gallery</a></p>"
            "</article>"
        )
    card_sections = []
    for group, cards in grouped_cards.items():
        card_sections.append(
            f"<section><h2>{html.escape(group)} ({len(cards)})</h2>"
            f"<div class='grid'>{''.join(cards)}</div></section>"
        )
    document = f"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>V5 Epoch {epoch} representative sample rechecks</title>
<style>
body {{ margin:0;padding:24px;background:#0e1117;color:#e6edf3;
font:14px/1.45 system-ui,sans-serif }} a {{ color:#58a6ff }}
.wrap {{ overflow-x:auto }} table {{ width:100%;border-collapse:collapse }}
th,td {{ padding:8px 10px;border-bottom:1px solid #30363d;text-align:right }}
th:nth-child(-n+4),td:nth-child(-n+4) {{ text-align:left }}
.grid {{ display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));
gap:16px;margin-top:24px }} .card {{ background:#161b22;border:1px solid #30363d;
border-radius:10px;padding:12px }} .card img {{ width:100%;border-radius:6px }}
</style></head>
<body><h1>V5 Epoch {epoch} representative sample rechecks</h1>
<p>{len(rows)} validation samples · K=1 · local-route origin (0,0)</p>
<p><a href="flat_gallery/index.html">한 폴더 슬라이드 뷰어 열기</a></p>
<div class="wrap"><table><thead><tr>
<th>idx</th><th>sample</th><th>category</th><th>group</th>
<th>rank</th><th>similarity</th><th>ADE/FDE @1s</th>
<th>ADE/FDE @2s</th><th>ADE/FDE @4s</th><th>lat MAE m</th>
<th>long MAE m</th><th>yaw MAE °</th><th>first x pred/GT m</th>
<th>origin mismatch</th><th>lateral flips</th>
</tr></thead><tbody>{''.join(table_rows)}</tbody></table></div>
{''.join(card_sections)}</body></html>"""
    (output_dir / "index.html").write_text(document, encoding="utf-8")


def main() -> None:
    args = parse_args()
    source = json.loads(args.source_summary.read_text(encoding="utf-8"))
    requested = source.get("samples", [])
    if not requested:
        raise ValueError(f"{args.source_summary}: no samples")
    if args.max_samples > 0:
        requested = requested[: args.max_samples]

    device = torch.device(
        "cpu" if args.cpu or not torch.cuda.is_available() else "cuda"
    )
    checkpoint = torch.load(args.checkpoint, map_location=device, weights_only=False)
    epoch = int(checkpoint["epoch"]) + 1
    model = MultiViewTemporalTrajectoryPlannerV5(
        ModelConfig(**checkpoint["model_config"])
    ).to(device)
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    splits = load_split_manifest(args.split_manifest)
    datasets: dict[str, PlannerDataset] = {}
    results: list[dict[str, Any]] = []
    args.output_dir.mkdir(parents=True, exist_ok=True)

    for source_row in requested:
        split = str(source_row["split"])
        if split not in datasets:
            datasets[split] = PlannerDataset(
                args.data_root,
                splits[split],
                blackout_weight=1.0,
                allow_legacy_target_fields=args.allow_legacy_target_fields,
                photometric_augmentation=False,
            )
        dataset = datasets[split]
        sample_index = int(source_row["dataset_sample_index"])
        sample = dataset[sample_index]
        if (
            sample["run_id"] != source_row["run_id"]
            or int(sample["sample_id"]) != int(source_row["sample_id"])
        ):
            raise RuntimeError(
                f"sample identity changed at {split}[{sample_index}]: "
                f"{sample['run_id']}/{sample['sample_id']}"
            )

        batch = make_batch(sample, device)
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

        name = str(source_row.get("name", f"sample_{sample_index}"))
        category = str(source_row.get("category", "representative"))
        sample_dir = args.output_dir / f"{sample_index}_{name}"
        collector = TopOutlierCollector(1)
        collector.update(outputs, batch, diagnostics)
        exported = collector.export(sample_dir, epoch)
        image_name = Path(exported["manifest"])
        manifest = json.loads(image_name.read_text(encoding="utf-8"))
        rendered = manifest["groups"]["overall"][0]["image"]
        result = {
            "checkpoint": str(args.checkpoint.resolve()),
            "checkpoint_epoch": epoch,
            "split": split,
            "dataset_sample_index": sample_index,
            "run_id": str(batch["run_id"][0]),
            "sample_id": int(batch["sample_id"][0]),
            "gps_blackout": bool(batch["gps_blackout"][0]),
            "name": name,
            "category": category,
            "group": str(source_row.get("group", category)),
            "anchor_name": str(source_row.get("anchor_name", name)),
            "anchor_dataset_sample_index": int(
                source_row.get("anchor_dataset_sample_index", sample_index)
            ),
            "similarity_rank": int(source_row.get("similarity_rank", 0)),
            "similarity": source_row.get("similarity", {"score": 0.0}),
            "kinematic_state": source_row.get("kinematic_state"),
            "camera_detection": source_row.get("camera_detection"),
            **_sample_metrics(diagnostics),
            "gallery": f"{sample_dir.name}/epoch_{epoch:03d}/index.html",
            "image": f"{sample_dir.name}/epoch_{epoch:03d}/{rendered}",
            "result_file": f"{sample_dir.name}/sample_result.json",
        }
        (sample_dir / "sample_result.json").write_text(
            json.dumps(result, indent=2), encoding="utf-8"
        )
        results.append(result)
        print(
            json.dumps(
                {
                    "sample": sample_index,
                    "name": name,
                    "ade_m": result["ade_m"],
                    "fde_m": result["fde_m"],
                },
                sort_keys=True,
            ),
            flush=True,
        )

    summary = {
        "checkpoint": str(args.checkpoint.resolve()),
        "checkpoint_epoch": epoch,
        "sample_count": len(results),
        "device": str(device),
        "samples": results,
    }
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    _write_index(args.output_dir, epoch, results)
    slideshow = write_flat_slideshow(args.output_dir, epoch, results)
    print(
        json.dumps(
            {
                "complete": True,
                "sample_count": len(results),
                "slideshow": str(slideshow),
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
