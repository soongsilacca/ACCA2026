#!/usr/bin/env python3
"""Append a directed MGeo connector from a CSV path's end back to its start."""

import argparse
import csv
import importlib.util
import math
import os
import shutil

import numpy as np
from scipy.ndimage import gaussian_filter1d


SAMPLE_INTERVAL = 0.1
SMOOTH_DISTANCE = 2.0
DIRECT_CLOSURE_DISTANCE = 3.0
DIRECT_CLOSURE_SEARCH_POINTS = 400
DIRECT_CLOSURE_MAX_HEADING_ERROR = math.radians(45.0)


def load_planner(planner_script):
    spec = importlib.util.spec_from_file_location("global_path_planner_node", planner_script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.GlobalPathPlanner


def read_rows(path):
    with open(path, "r", newline="") as stream:
        reader = csv.DictReader(stream)
        return reader.fieldnames, list(reader)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--output", help="Output CSV; defaults to overwriting csv_path")
    parser.add_argument("--yaw-source", help="Original CSV whose endpoint odometry yaw should be used")
    parser.add_argument("--node-file", required=True)
    parser.add_argument("--link-file", required=True)
    parser.add_argument("--planner-script", required=True)
    args = parser.parse_args()

    fields, rows = read_rows(args.csv_path)
    points = np.asarray([[float(row["x"]), float(row["y"])] for row in rows])
    yaw_rows = read_rows(args.yaw_source)[1] if args.yaw_source else rows
    yaw = np.asarray([float(row["yaw"]) for row in yaw_rows])
    sample_count = min(100, len(yaw))
    start_yaw = float(np.median(yaw[:sample_count]))
    end_yaw = float(np.median(yaw[-sample_count:]))

    # Remove stationary GNSS drift at both ends. The extrema along the
    # odometry heading mark the first and last forward-driving samples.
    start_window = min(100, len(points))
    end_window = min(200, len(points))
    start_direction = np.asarray([math.cos(start_yaw), math.sin(start_yaw)])
    end_direction = np.asarray([math.cos(end_yaw), math.sin(end_yaw)])
    start_index = int(np.argmin(points[:start_window] @ start_direction))

    # A completed lap can contain a short overshoot or reverse-motion tail.
    # Prefer the last same-direction return to the stable start point instead
    # of choosing the forward-most tail sample. This avoids planning an entire
    # directed MGeo detour when the driven loop was already closed.
    geometry = gaussian_filter1d(points, sigma=3.0, axis=0, mode="nearest")
    geometry_delta = np.gradient(geometry, axis=0)
    geometry_yaw = np.arctan2(geometry_delta[:, 1], geometry_delta[:, 0])
    closure_begin = max(start_index + 10, len(points) - DIRECT_CLOSURE_SEARCH_POINTS)
    closure_candidates = np.arange(closure_begin, len(points))
    closure_distance = np.linalg.norm(points[closure_candidates] - points[start_index], axis=1)
    closure_heading_error = np.abs(np.arctan2(
        np.sin(geometry_yaw[closure_candidates] - geometry_yaw[start_index]),
        np.cos(geometry_yaw[closure_candidates] - geometry_yaw[start_index]),
    ))
    same_direction = closure_heading_error <= DIRECT_CLOSURE_MAX_HEADING_ERROR
    eligible = closure_candidates[same_direction]
    if len(eligible):
        direct_end_index = int(eligible[np.argmin(
            np.linalg.norm(points[eligible] - points[start_index], axis=1)
        )])
    else:
        direct_end_index = int(closure_candidates[np.argmin(closure_distance)])
    direct_gap = float(np.linalg.norm(points[direct_end_index] - points[start_index]))
    direct_closure = (
        direct_gap <= DIRECT_CLOSURE_DISTANCE
        and direct_end_index > start_index + len(points) // 2
    )
    if direct_closure:
        end_index = direct_end_index
    else:
        end_offset = len(points) - end_window
        end_index = end_offset + int(np.argmax(points[end_offset:] @ end_direction))
    if end_index <= start_index + 2:
        raise RuntimeError("Stationary trimming removed the whole path")
    main_rows = rows[start_index:end_index + 1]
    main_points = points[start_index:end_index + 1]

    connector_xy = np.empty((0, 2), dtype=float)
    link_sequence = []
    if not direct_closure:
        Planner = load_planner(args.planner_script)
        planner = Planner(args.node_file, args.link_file)
        connector, link_sequence = planner.plan_path(
            [main_points[-1, 0], main_points[-1, 1], 0.0],
            [main_points[0, 0], main_points[0, 1], 0.0],
            end_yaw, start_yaw, True, SAMPLE_INTERVAL,
        )
        if not connector:
            raise RuntimeError("No directed MGeo route connects path end to start")
        connector_xy = np.asarray([[point["x"], point["y"]] for point in connector])
        if np.linalg.norm(connector_xy[0] - main_points[-1]) < 1.0:
            connector_xy = connector_xy[1:]
        if len(connector_xy) and np.linalg.norm(connector_xy[-1] - main_points[0]) < 1.0:
            connector_xy = connector_xy[:-1]
    raw_points = np.vstack((main_points, connector_xy))

    # Uniform periodic resampling plus wrap-mode smoothing makes position and
    # tangent continuous at the final-to-first seam.
    closed_raw = np.vstack((raw_points, raw_points[0]))
    raw_step = np.linalg.norm(np.diff(closed_raw, axis=0), axis=1)
    raw_s = np.concatenate(([0.0], np.cumsum(raw_step)))
    sample_s = np.arange(0.0, raw_s[-1], SAMPLE_INTERVAL)
    smooth = np.column_stack([
        np.interp(sample_s, raw_s, closed_raw[:, axis]) for axis in range(2)
    ])
    smooth = np.column_stack([
        gaussian_filter1d(smooth[:, axis], SMOOTH_DISTANCE / SAMPLE_INTERVAL, mode="wrap")
        for axis in range(2)
    ])

    # Smoothing slightly shortens curved sections. Resample the smoothed loop
    # once more so the CSV itself, rather than only the pre-smoothed curve, has
    # a fixed 0.1 m sampling interval. A closed loop can only contain an integer
    # number of samples, so its circumference is divided by the nearest count.
    smooth_closed = np.vstack((smooth, smooth[0]))
    smooth_step = np.linalg.norm(np.diff(smooth_closed, axis=0), axis=1)
    smooth_s = np.concatenate(([0.0], np.cumsum(smooth_step)))
    output_count = max(3, int(round(smooth_s[-1] / SAMPLE_INTERVAL)))
    output_s = np.linspace(0.0, smooth_s[-1], output_count, endpoint=False)
    smooth = np.column_stack([
        np.interp(output_s, smooth_s, smooth_closed[:, axis]) for axis in range(2)
    ])
    all_points = np.vstack((smooth, smooth[0]))

    stamps = np.asarray([float(row["stamp"]) for row in main_rows]) if "stamp" in fields else None
    positive_dt = np.diff(stamps) if stamps is not None else np.asarray([])
    positive_dt = positive_dt[positive_dt > 0]
    dt = float(np.median(positive_dt)) if len(positive_dt) else 0.05
    main_tokens = [row.get("token", "0") for row in main_rows]
    raw_tokens = main_tokens + [main_tokens[-1]] * len(connector_xy)
    raw_output_s = output_s * (raw_s[-1] / smooth_s[-1])
    token_indices = np.clip(
        np.searchsorted(raw_s, raw_output_s, side="right") - 1,
        0,
        len(raw_tokens) - 1,
    )
    output_rows = []
    heading = np.arctan2(
        np.roll(smooth[:, 1], -1) - smooth[:, 1],
        np.roll(smooth[:, 0], -1) - smooth[:, 0],
    )
    heading = np.concatenate((heading, heading[:1]))
    distance = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(all_points, axis=0), axis=1))))
    for index, point in enumerate(all_points):
        row = {field: "" for field in fields}
        row["x"], row["y"] = "{:.9f}".format(point[0]), "{:.9f}".format(point[1])
        if "stamp" in fields:
            row["stamp"] = "{:.6f}".format(stamps[0] + index * dt)
        if "token" in fields:
            source_index = token_indices[index] if index < len(output_s) else token_indices[0]
            row["token"] = raw_tokens[source_index]
        if "yaw" in fields:
            row["yaw"] = "{:.9f}".format(heading[index])
        if "s" in fields:
            row["s"] = "{:.6f}".format(distance[index])
        output_rows.append(row)

    output = args.output or args.csv_path
    backup = args.csv_path + ".open"
    if os.path.abspath(output) == os.path.abspath(args.csv_path) and not os.path.exists(backup):
        shutil.copy2(args.csv_path, backup)
    temporary = output + ".tmp"
    with open(temporary, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(output_rows)
    os.replace(temporary, output)
    if os.path.exists(backup):
        print("Backup:", backup)
    print("Output:", output)
    print("Trimmed source range: {}..{}".format(start_index, end_index))
    if direct_closure:
        print("Direct lap closure: {:.3f} m gap; overshoot tail removed: {} points".format(
            direct_gap, len(points) - 1 - end_index
        ))
    else:
        print("Connector links:", ", ".join(link_sequence))
    print("Output points:", len(all_points))
    print("Requested sampling interval: {:.3f} m".format(SAMPLE_INTERVAL))
    output_step = np.linalg.norm(np.diff(all_points, axis=0), axis=1)
    print(
        "Actual sampling interval: min={:.6f} mean={:.6f} max={:.6f} m".format(
            output_step.min(), output_step.mean(), output_step.max()
        )
    )
    print("Closure gap: {:.6f} m".format(np.linalg.norm(all_points[-1] - all_points[0])))
    segment_heading = np.arctan2(np.diff(all_points[:, 1]), np.diff(all_points[:, 0]))
    turn = np.abs(np.arctan2(np.sin(np.diff(segment_heading)), np.cos(np.diff(segment_heading))))
    print("Maximum turn per point: {:.3f} deg".format(np.degrees(turn.max())))


if __name__ == "__main__":
    main()
