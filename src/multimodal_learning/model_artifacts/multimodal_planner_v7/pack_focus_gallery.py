from __future__ import annotations

import argparse
import html
import json
import shutil
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Pack obstacle, large-Δd and stopped V7 samples for review."
    )
    parser.add_argument("--source-summary", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--large-delta-threshold-m", type=float, default=0.75)
    return parser.parse_args()


def _is_obstacle(row: dict[str, Any]) -> bool:
    text = f"{row.get('group', '')} {row.get('category', '')}".lower()
    return "obstacle" in text


def _copy_group(
    source_root: Path,
    output_dir: Path,
    name: str,
    rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    group_dir = output_dir / name
    group_dir.mkdir(parents=True, exist_ok=True)
    packed = []
    for rank, row in enumerate(rows, 1):
        source = source_root / row["image"]
        destination_name = (
            f"{rank:03d}_{int(row['dataset_sample_index']):05d}_{source.name}"
        )
        destination = group_dir / destination_name
        shutil.copy2(source, destination)
        copy = dict(row)
        copy["focused_image"] = f"{name}/{destination_name}"
        packed.append(copy)
    return packed


def main() -> None:
    args = parse_args()
    payload = json.loads(args.source_summary.read_text(encoding="utf-8"))
    rows = list(payload["samples"])
    source_root = args.source_summary.parent
    args.output_dir.mkdir(parents=True, exist_ok=True)

    large_delta = [
        row
        for row in rows
        if max(
            float(row["target_max_abs_lateral_residual_m"]),
            float(row["predicted_max_abs_lateral_residual_m"]),
        )
        >= args.large_delta_threshold_m
    ]
    large_delta.sort(
        key=lambda row: max(
            float(row["target_max_abs_lateral_residual_m"]),
            float(row["predicted_max_abs_lateral_residual_m"]),
        ),
        reverse=True,
    )
    obstacles = [row for row in rows if _is_obstacle(row)]
    obstacles.sort(
        key=lambda row: (
            not bool(row.get("avoidance", False)),
            -float(row["target_max_abs_lateral_residual_m"]),
        )
    )
    stopped = [
        row
        for row in rows
        if row.get("motion_state_target") == "STOP"
        or row.get("motion_state_prediction") == "STOP"
    ]
    stopped.sort(
        key=lambda row: (
            row.get("motion_state_target") != "STOP",
            -float(row.get("motion_state_confidence", 0.0)),
        )
    )
    groups = {
        "01_large_delta": large_delta,
        "02_obstacles": obstacles,
        "03_stopped": stopped,
    }
    packed_groups = {
        name: _copy_group(source_root, args.output_dir, name, group_rows)
        for name, group_rows in groups.items()
    }

    sections = []
    for name, group_rows in packed_groups.items():
        cards = []
        for row in group_rows:
            cards.append(
                "<article class='card'>"
                f"<a href='{html.escape(row['focused_image'])}'>"
                f"<img loading='lazy' src='{html.escape(row['focused_image'])}'></a>"
                f"<h3>#{row['dataset_sample_index']} {html.escape(row['name'])}</h3>"
                f"<p>{html.escape(row['group'])}</p>"
                f"<p>state {row['motion_state_target']}→{row['motion_state_prediction']} "
                f"({row['motion_state_confidence']:.3f}) · avoidance={str(row['avoidance']).lower()}</p>"
                f"<p>max |Δd| GT/Pred "
                f"{row['target_max_abs_lateral_residual_m']:.2f}/"
                f"{row['predicted_max_abs_lateral_residual_m']:.2f}m · "
                f"Δd MAE {row['route_residual_mae_m']:.2f}m</p>"
                "</article>"
            )
        sections.append(
            f"<section><h2>{html.escape(name)} ({len(group_rows)})</h2>"
            f"<div class='grid'>{''.join(cards)}</div></section>"
        )
    document = f"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>V7 focused driving review</title><style>
body{{margin:0;padding:24px;background:#070b12;color:#e2e8f0;font:15px/1.45 system-ui,sans-serif}}
a{{color:#60a5fa}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(560px,1fr));gap:16px}}
.card{{background:#111827;border:1px solid #334155;border-radius:12px;padding:12px}}
.card img{{width:100%;border-radius:8px}}h2{{margin-top:36px}}</style></head>
<body><h1>V7 focused review: obstacle / large Δd / stopped</h1>
<p>같은 sample이 여러 진단 기준에 해당하면 폴더 간 중복될 수 있습니다.</p>
{''.join(sections)}</body></html>"""
    (args.output_dir / "index.html").write_text(document, encoding="utf-8")
    manifest = {
        "source_summary": str(args.source_summary.resolve()),
        "large_delta_threshold_m": args.large_delta_threshold_m,
        "groups": packed_groups,
        "counts": {name: len(value) for name, value in packed_groups.items()},
    }
    (args.output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2),
        encoding="utf-8",
    )
    print(json.dumps({"complete": True, "counts": manifest["counts"], "gallery": str(args.output_dir / "index.html")}))


if __name__ == "__main__":
    main()
