#!/usr/bin/env python3
"""Aggregate closed-loop summary.csv and render the report comparison chart."""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


RATE_COLUMNS = (
    ("route_completion", "Route Completion"),
    ("collision", "Collision (lower is better)"),
    ("signal_success", "Signal"),
    ("static_avoid_success", "Static Avoid"),
    ("dynamic_stop_success", "Dynamic Stop"),
)


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("results", nargs="?", default="closed_loop_results")
    args = parser.parse_args()
    root = Path(args.results).expanduser().resolve()
    source = root / "summary.csv"
    if not source.exists():
        raise SystemExit("summary.csv not found: %s" % source)
    with source.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    grouped = defaultdict(list)
    total_by_model = defaultdict(int)
    for row in rows:
        model = row["model"]
        total_by_model[model] += 1
        # A roslaunch/user interruption is not a completed closed-loop trial.
        reason = row.get("failure_reason", "").strip().upper()
        if reason == "INTERRUPTED" or reason.startswith("INVALID_"):
            continue
        grouped[model].append(row)
    output_rows = []
    for model, items in sorted(grouped.items()):
        result = {
            "model": model,
            "total_runs": total_by_model[model],
            "valid_runs": len(items),
            "excluded_interrupted": total_by_model[model] - len(items),
        }
        for key, _ in RATE_COLUMNS:
            values = [number(item.get(key)) for item in items]
            values = [value for value in values if value is not None]
            result[key] = float(np.mean(values)) if values else float("nan")
        for key in ("mean_cte", "max_cte", "mean_latency", "p95_latency"):
            values = [number(item.get(key)) for item in items]
            values = [value for value in values if value is not None]
            result[key] = float(np.mean(values)) if values else float("nan")
        output_rows.append(result)
    aggregate = root / "aggregate.csv"
    fields = ["model", "total_runs", "valid_runs", "excluded_interrupted"] + [key for key, _ in RATE_COLUMNS] + [
        "mean_cte", "max_cte", "mean_latency", "p95_latency"]
    with aggregate.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(output_rows)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Wrote %s (matplotlib unavailable; chart skipped)" % aggregate)
        return
    models = [row["model"] for row in output_rows]
    x = np.arange(len(models)); width = 0.15
    fig, axis = plt.subplots(figsize=(11, 5.5))
    for offset, (key, label) in enumerate(RATE_COLUMNS):
        values = [100.0*row[key] for row in output_rows]
        axis.bar(x + (offset-2)*width, values, width, label=label)
    axis.set_xticks(x); axis.set_xticklabels(models)
    axis.set_ylabel("Rate (%)")
    axis.set_ylim(0.0, 105.0); axis.grid(axis="y", alpha=0.25)
    axis.legend(ncol=3); fig.tight_layout()
    chart = root / "model_comparison.png"
    fig.savefig(str(chart), dpi=180); plt.close(fig)
    print("Wrote %s and %s" % (aggregate, chart))


if __name__ == "__main__":
    main()
