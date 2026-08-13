from __future__ import annotations

import argparse
import html
import json
import re
import shutil
from pathlib import Path
from typing import Any


def _safe_name(value: str) -> str:
    return re.sub(r"[^0-9A-Za-z가-힣._-]+", "_", value).strip("_") or "sample"


def write_flat_slideshow(
    output_dir: Path,
    epoch: int,
    rows: list[dict[str, Any]],
) -> Path:
    gallery_dir = output_dir / "flat_gallery"
    image_dir = gallery_dir / "images"
    image_dir.mkdir(parents=True, exist_ok=True)
    slides = []
    for order, row in enumerate(rows, start=1):
        source = output_dir / row["image"]
        suffix = source.suffix.lower() or ".png"
        filename = (
            f"{order:03d}_{_safe_name(str(row.get('group', row['category'])))}_"
            f"{_safe_name(str(row['name']))}_idx{row['dataset_sample_index']}{suffix}"
        )
        destination = image_dir / filename
        shutil.copy2(source, destination)
        horizons = row["horizon_metrics"]
        slides.append(
            {
                "order": order,
                "image": f"images/{filename}",
                "name": row["name"],
                "group": row.get("group", row["category"]),
                "category": row["category"],
                "dataset_sample_index": row["dataset_sample_index"],
                "run_id": row["run_id"],
                "sample_id": row["sample_id"],
                "gps_blackout": row["gps_blackout"],
                "ade_4s_m": horizons["4s"]["ade_m"],
                "fde_4s_m": horizons["4s"]["fde_m"],
                "similarity_rank": row.get("similarity_rank", 0),
            }
        )

    data_json = json.dumps(slides, ensure_ascii=False).replace("</", "<\\/")
    document = f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>V5 Epoch {epoch} slide viewer</title>
<style>
* {{ box-sizing:border-box }}
body {{ margin:0;background:#0d1117;color:#e6edf3;font:14px/1.45 system-ui,sans-serif }}
header {{ position:sticky;top:0;z-index:2;display:flex;align-items:center;gap:10px;
padding:10px 14px;background:#161b22;border-bottom:1px solid #30363d }}
button,select {{ background:#21262d;color:#e6edf3;border:1px solid #30363d;
border-radius:6px;padding:8px 12px }} button {{ cursor:pointer }}
#counter {{ min-width:84px;text-align:center;font-weight:700 }}
#title {{ flex:1;white-space:nowrap;overflow:hidden;text-overflow:ellipsis }}
main {{ display:grid;grid-template-columns:minmax(0,1fr) 270px;gap:14px;padding:14px }}
.stage {{ min-height:calc(100vh - 86px);display:flex;align-items:center;
justify-content:center;background:#010409;border:1px solid #30363d;border-radius:8px }}
.stage img {{ display:block;max-width:100%;max-height:calc(100vh - 92px);object-fit:contain }}
aside {{ padding:14px;background:#161b22;border:1px solid #30363d;border-radius:8px }}
aside dt {{ color:#8b949e;margin-top:10px }} aside dd {{ margin:2px 0 0;word-break:break-all }}
.hint {{ color:#8b949e;margin-top:20px }}
@media (max-width:900px) {{ main {{ grid-template-columns:1fr }} aside {{ order:-1 }}
.stage {{ min-height:auto }} }}
</style></head><body>
<header>
<button id="prev" title="이전 (←)">◀ 이전</button>
<span id="counter"></span>
<button id="next" title="다음 (→)">다음 ▶</button>
<select id="group"><option value="">전체 그룹</option></select>
<span id="title"></span>
</header>
<main><div class="stage"><img id="image" alt="trajectory sample"></div>
<aside><h2 id="name"></h2><dl id="meta"></dl>
<p class="hint">←/→ 방향키로 이동 · Home/End 처음/끝 · 그림 클릭 시 다음</p>
</aside></main>
<script>
const allSlides = {data_json};
let slides = allSlides.slice();
let index = 0;
const byId = id => document.getElementById(id);
const groups = [...new Set(allSlides.map(x => x.group))];
for (const group of groups) {{
  const option = document.createElement("option");
  option.value = group; option.textContent = group; byId("group").appendChild(option);
}}
function render() {{
  if (!slides.length) return;
  index = (index + slides.length) % slides.length;
  const x = slides[index];
  byId("image").src = x.image;
  byId("counter").textContent = `${{index + 1}} / ${{slides.length}}`;
  byId("title").textContent = `${{x.group}} · ${{x.name}}`;
  byId("name").textContent = x.name;
  byId("meta").innerHTML =
    `<dt>그룹</dt><dd>${{x.group}}</dd>` +
    `<dt>dataset index</dt><dd>${{x.dataset_sample_index}}</dd>` +
    `<dt>run / sample</dt><dd>${{x.run_id}} / ${{x.sample_id}}</dd>` +
    `<dt>ADE / FDE @4s</dt><dd>${{x.ade_4s_m.toFixed(3)}} / ${{x.fde_4s_m.toFixed(3)}} m</dd>` +
    `<dt>similarity rank</dt><dd>${{x.similarity_rank}}</dd>` +
    `<dt>GPS blackout</dt><dd>${{x.gps_blackout}}</dd>`;
  location.hash = String(x.order);
}}
function move(delta) {{ index += delta; render(); }}
byId("prev").onclick = () => move(-1);
byId("next").onclick = () => move(1);
byId("image").onclick = () => move(1);
byId("group").onchange = event => {{
  slides = event.target.value
    ? allSlides.filter(x => x.group === event.target.value) : allSlides.slice();
  index = 0; render();
}};
document.addEventListener("keydown", event => {{
  if (event.key === "ArrowLeft") move(-1);
  else if (event.key === "ArrowRight" || event.key === " ") move(1);
  else if (event.key === "Home") {{ index = 0; render(); }}
  else if (event.key === "End") {{ index = slides.length - 1; render(); }}
}});
const initialOrder = Number(location.hash.slice(1));
if (initialOrder) {{
  const found = slides.findIndex(x => x.order === initialOrder);
  if (found >= 0) index = found;
}}
render();
</script></body></html>"""
    gallery_dir.mkdir(parents=True, exist_ok=True)
    (gallery_dir / "index.html").write_text(document, encoding="utf-8")
    (gallery_dir / "manifest.json").write_text(
        json.dumps(
            {
                "checkpoint_epoch": epoch,
                "image_count": len(slides),
                "images_directory": "images",
                "slides": slides,
            },
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    return gallery_dir / "index.html"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect recheck images into one folder and build a slide viewer."
    )
    parser.add_argument("--summary", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    summary = json.loads(args.summary.read_text(encoding="utf-8"))
    rows = list(summary.get("samples", []))
    if not rows:
        raise ValueError(f"{args.summary}: no samples")
    epoch = int(summary["checkpoint_epoch"])
    index = write_flat_slideshow(args.summary.parent, epoch, rows)
    print(
        json.dumps(
            {
                "complete": True,
                "image_count": len(rows),
                "gallery": str(index),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
