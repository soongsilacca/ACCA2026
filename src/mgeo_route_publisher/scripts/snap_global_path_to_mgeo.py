#!/usr/bin/env python3
"""Map-match a recorded global-path CSV onto MGeo link centerlines."""

import argparse
import csv
import json
import math
import os
import shutil

import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import gaussian_filter1d
from scipy.signal import savgol_filter


def angle_difference(a, b):
    return np.arctan2(np.sin(a - b), np.cos(a - b))


def load_segments(link_file):
    with open(link_file, "r") as stream:
        links = json.load(stream)
    starts, ends, link_ids, segment_indices = [], [], [], []
    from_nodes = [link.get("from_node_idx") for link in links]
    to_nodes = [link.get("to_node_idx") for link in links]
    for link_index, link in enumerate(links):
        points = np.asarray(link.get("points", []), dtype=np.float64)
        if len(points) < 2:
            continue
        for segment_index in range(len(points) - 1):
            start, end = points[segment_index, :2], points[segment_index + 1, :2]
            if np.linalg.norm(end - start) < 1e-4:
                continue
            starts.append(start)
            ends.append(end)
            link_ids.append(link_index)
            segment_indices.append(segment_index)
    return (
        np.asarray(starts), np.asarray(ends),
        np.asarray(link_ids, dtype=np.int32), np.asarray(segment_indices, dtype=np.int32),
        np.asarray(from_nodes, dtype=object), np.asarray(to_nodes, dtype=object),
    )


def load_csv(path):
    with open(path, "r", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fields = reader.fieldnames
    if not fields or "x" not in fields or "y" not in fields:
        raise ValueError("CSV must contain x and y columns")
    points = np.asarray([[float(row["x"]), float(row["y"])] for row in rows])
    # The recorded yaw can be unreliable while stationary and may use a
    # sensor-specific convention. Path order is the authoritative route
    # direction, so derive matching headings from smoothed x/y geometry.
    window = min(11, len(points) if len(points) % 2 else len(points) - 1)
    heading_points = points
    if window >= 5:
        heading_points = np.column_stack([
            savgol_filter(points[:, axis], window, 2, mode="interp")
            for axis in range(2)
        ])
    delta = np.gradient(heading_points, axis=0)
    headings = np.arctan2(delta[:, 1], delta[:, 0])
    return fields, rows, points, headings


def candidate_projections(points, starts, ends, candidate_count):
    midpoints = (starts + ends) * 0.5
    tree = cKDTree(midpoints)
    _, indices = tree.query(points, k=min(candidate_count, len(midpoints)))
    if indices.ndim == 1:
        indices = indices[:, None]
    seg_start, seg_end = starts[indices], ends[indices]
    vector = seg_end - seg_start
    length_sq = np.sum(vector * vector, axis=2)
    offset = points[:, None, :] - seg_start
    fraction = np.clip(np.sum(offset * vector, axis=2) / length_sq, 0.0, 1.0)
    projected = seg_start + fraction[..., None] * vector
    distance = np.linalg.norm(points[:, None, :] - projected, axis=2)
    heading = np.arctan2(vector[:, :, 1], vector[:, :, 0])
    return indices, projected, distance, heading, fraction


def viterbi_match(
    points, headings, candidates, projected, distances, segment_heading,
    link_ids, from_nodes, to_nodes,
):
    count, width = candidates.shape
    backtrack = np.zeros((count, width), dtype=np.int16)
    heading_error = angle_difference(segment_heading, headings[:, None])
    # Roughly 2 m lateral error and 45 degree heading error have comparable cost.
    emission = distances ** 2 + 3.0 * (1.0 - np.cos(heading_error))
    cost = emission[0]
    observed_delta = np.diff(points, axis=0)
    for index in range(1, count):
        previous = projected[index - 1]
        current = projected[index]
        map_delta = current[None, :, :] - previous[:, None, :]
        motion_error = np.linalg.norm(map_delta - observed_delta[index - 1], axis=2)
        previous_links = link_ids[candidates[index - 1]][:, None]
        current_links = link_ids[candidates[index]][None, :]
        switch = previous_links != current_links
        connected = (~switch) | (to_nodes[previous_links] == from_nodes[current_links])
        # Strong continuity keeps noisy GNSS samples from hopping between
        # parallel lane centerlines. Connected links still transition because
        # their projected endpoints are spatially continuous.
        transition = 6.0 * motion_error ** 2 + 0.2 * switch + (~connected) * 1.0e6
        total = cost[:, None] + transition
        best_previous = np.argmin(total, axis=0)
        backtrack[index] = best_previous
        cost = emission[index] + total[best_previous, np.arange(width)]
        # Keep values numerically small for long recordings.
        cost -= cost.min()
    state = np.empty(count, dtype=np.int16)
    state[-1] = int(np.argmin(cost))
    for index in range(count - 1, 0, -1):
        state[index - 1] = backtrack[index, state[index]]
    row = np.arange(count)
    return candidates[row, state], projected[row, state], distances[row, state]


def remove_backward_jitter(points, segment_ids, link_ids, segment_indices):
    """Hold position for small backward matches on the same directed MGeo link."""
    output = points.copy()
    for index in range(1, len(output)):
        previous_segment, segment = segment_ids[index - 1], segment_ids[index]
        if link_ids[previous_segment] != link_ids[segment]:
            continue
        if segment_indices[segment] < segment_indices[previous_segment]:
            output[index] = output[index - 1]
    return output


def smooth_mgeo_approximation(
    points, headings, candidates, projected, distances, segment_heading,
    correction_sigma=15.0, max_correction=1.5,
):
    """Preserve the driven branch while gently attracting it toward MGeo.

    Independent nearest-link corrections can jump between parallel lanes, while
    hard graph connectivity can select the wrong branch at imperfect MGeo
    junctions.  Smoothing the correction vector avoids both failure modes.
    """
    window = min(11, len(points) if len(points) % 2 else len(points) - 1)
    if window >= 5:
        base = np.column_stack([
            savgol_filter(points[:, axis], window, 2, mode="interp")
            for axis in range(2)
        ])
    else:
        base = points.copy()
    heading_error = angle_difference(segment_heading, headings[:, None])
    emission = distances ** 2 + 2.0 * (1.0 - np.cos(heading_error))
    state = np.argmin(emission, axis=1)
    row = np.arange(len(points))
    raw_correction = projected[row, state] - points
    correction = gaussian_filter1d(raw_correction, correction_sigma, axis=0, mode="nearest")
    magnitude = np.linalg.norm(correction, axis=1)
    scale = np.minimum(1.0, max_correction / np.maximum(magnitude, 1e-6))
    correction *= scale[:, None]
    output = base + correction
    return output, np.linalg.norm(output - points, axis=1)


def update_rows(fields, rows, points):
    delta = np.gradient(points, axis=0)
    yaw = np.arctan2(delta[:, 1], delta[:, 0])
    step = np.linalg.norm(np.diff(points, axis=0), axis=1)
    distance = np.concatenate(([0.0], np.cumsum(step)))
    for index, row in enumerate(rows):
        row["x"] = "{:.9f}".format(points[index, 0])
        row["y"] = "{:.9f}".format(points[index, 1])
        if "yaw" in fields:
            row["yaw"] = "{:.9f}".format(yaw[index])
        if "s" in fields:
            row["s"] = "{:.6f}".format(distance[index])


def save_plot(path, original, snapped, starts, ends):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    margin = 30.0
    lower = original.min(axis=0) - margin
    upper = original.max(axis=0) + margin
    nearby = (
        (starts[:, 0] >= lower[0]) & (starts[:, 0] <= upper[0]) &
        (starts[:, 1] >= lower[1]) & (starts[:, 1] <= upper[1])
    )
    map_segments = np.stack((starts[nearby], ends[nearby]), axis=1)
    figure, axes = plt.subplots(1, 2, figsize=(16, 8), constrained_layout=True)
    for axis in axes:
        axis.add_collection(LineCollection(map_segments, colors="#b8c2cc", linewidths=0.45, alpha=0.55, label="MGeo links"))
        axis.plot(original[:, 0], original[:, 1], color="#ef4444", linewidth=0.8, alpha=0.55, label="Recorded")
        axis.plot(snapped[:, 0], snapped[:, 1], color="#2563eb", linewidth=1.25, label="MGeo matched")
        axis.scatter(original[0, 0], original[0, 1], s=45, color="#16a34a", zorder=4, label="Start")
        axis.scatter(original[-1, 0], original[-1, 1], s=45, color="#111827", zorder=4, label="End")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, linewidth=0.3, alpha=0.4)
        axis.set_xlabel("map x [m]")
        axis.set_ylabel("map y [m]")
    axes[0].set_title("Full path: recorded vs MGeo matched")
    axes[0].set_xlim(lower[0], upper[0]); axes[0].set_ylim(lower[1], upper[1])
    # Zoom around the largest original-to-snapped correction.
    correction = np.linalg.norm(original - snapped, axis=1)
    center = original[int(np.argmax(correction))]
    axes[1].set_title("Largest correction area")
    axes[1].set_xlim(center[0] - 25.0, center[0] + 25.0)
    axes[1].set_ylim(center[1] - 25.0, center[1] + 25.0)
    handles, labels = axes[0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="upper center", ncol=5)
    figure.savefig(path, dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv")
    parser.add_argument("--output", help="Defaults to overwriting input after creating .original backup")
    parser.add_argument("--link-file", required=True)
    parser.add_argument("--candidates", type=int, default=16)
    parser.add_argument("--plot", help="Save original/MGeo/snapped comparison PNG")
    parser.add_argument("--mode", choices=("smooth", "topology"), default="smooth")
    parser.add_argument("--correction-sigma", type=float, default=15.0)
    parser.add_argument("--max-correction", type=float, default=1.5)
    args = parser.parse_args()
    fields, rows, points, headings = load_csv(args.input_csv)
    starts, ends, link_ids, segment_indices, from_nodes, to_nodes = load_segments(args.link_file)
    candidate_data = candidate_projections(points, starts, ends, args.candidates)
    candidates, projected, distances, segment_heading, _ = candidate_data
    if args.mode == "topology":
        matched_segments, snapped, matched_distance = viterbi_match(
            points, headings, candidates, projected, distances, segment_heading,
            link_ids, from_nodes, to_nodes,
        )
        snapped = remove_backward_jitter(snapped, matched_segments, link_ids, segment_indices)
    else:
        snapped, matched_distance = smooth_mgeo_approximation(
            points, headings, candidates, projected, distances, segment_heading,
            args.correction_sigma, args.max_correction,
        )
    update_rows(fields, rows, snapped)
    output = args.output or args.input_csv
    if os.path.abspath(output) == os.path.abspath(args.input_csv):
        backup = args.input_csv + ".original"
        if not os.path.exists(backup):
            shutil.copy2(args.input_csv, backup)
            print("Backup:", backup)
    temporary = output + ".tmp"
    with open(temporary, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    os.replace(temporary, output)
    if args.plot:
        save_plot(args.plot, points, snapped, starts, ends)
        print("Plot:", args.plot)
    print("Output:", output)
    print("Points:", len(points))
    print("Mean/max applied correction: {:.3f} / {:.3f} m".format(matched_distance.mean(), matched_distance.max()))
    print("Result length: {:.3f} m".format(np.linalg.norm(np.diff(snapped, axis=0), axis=1).sum()))


if __name__ == "__main__":
    main()
