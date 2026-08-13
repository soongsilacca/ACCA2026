#!/usr/bin/env python3
import argparse
from pathlib import Path
import re

import numpy as np

from multimodal_learning.bag_reader import read_bag
from multimodal_learning.bag_tokens import mgeo_tokens_from_msg, route_tokens_from_msg
from multimodal_learning.coordinate_transform import pose_from_odometry
from multimodal_learning.io_utils import load_yaml, package_config
from multimodal_learning.lidar_bev import pointcloud2_to_xyz, points_to_bev
from multimodal_learning.sensor_preprocess import image_from_msg, ego_localization_vector
from multimodal_learning.synchronizer import synchronize
from multimodal_learning.temporal_sampler import history_indices
from multimodal_learning.trajectory_labeler import auxiliary_labels, future_trajectory


def main():
    parser = argparse.ArgumentParser(description="Extract V2 samples from a ROS1 bag")
    parser.add_argument("bag")
    parser.add_argument("--topics", default=package_config("topics.yaml"))
    parser.add_argument("--dataset-config", default=package_config("dataset.yaml"))
    parser.add_argument("--output")
    parser.add_argument("--max-samples", type=int)
    parser.add_argument(
        "--prefix",
        help="Output filename prefix (default: bag filename; prevents multi-bag overwrites)",
    )
    args = parser.parse_args()
    topics, cfg = load_yaml(args.topics), load_yaml(args.dataset_config)
    output = Path(args.output or cfg["output_dir"]).expanduser()
    output.mkdir(parents=True, exist_ok=True)

    stream_names = (
        "camera_front", "camera_left", "camera_right", "lidar", "odom", "imu",
        "vehicle_status", "gps", "local_route", "mgeo_token",
    )
    names = {name: topics[name + "_topic"] for name in stream_names}
    streams = read_bag(args.bag, names)
    rows = list(synchronize(
        streams["odom"],
        {key: value for key, value in streams.items() if key != "odom"},
        cfg["sync_tolerance"],
    ))
    if not rows:
        raise RuntimeError("No synchronized V2 rows found; check required topics and timestamps")

    poses = np.asarray([pose_from_odometry(row["reference"]) for row in rows], np.float32)
    velocity_scale = float(cfg.get("vehicle_velocity_scale", 1.0))
    speeds = np.asarray([
        np.hypot(row["vehicle_status"].velocity.x, row["vehicle_status"].velocity.y)
        * velocity_scale for row in rows
    ], np.float32)
    stamps = [row["stamp"] for row in rows]
    history = cfg["history_frames"]
    horizon = cfg["prediction_horizon"]
    written, last_sample_stamp = 0, float("-inf")

    for current, row in enumerate(rows):
        if row["stamp"] - last_sample_stamp < cfg["sample_period"] - 1e-4:
            continue
        indices = history_indices(
            stamps, current, history, cfg["history_interval"], cfg["sync_tolerance"]
        )
        label = future_trajectory(
            poses, speeds, stamps, current, horizon,
            cfg["prediction_interval"], cfg["sync_tolerance"],
        )
        if indices is None or label is None:
            continue

        sample = {key: [] for key in (
            "camera_front", "camera_left", "camera_right", "lidar", "ego"
        )}
        try:
            for index in indices:
                history_row = rows[index]
                sample["camera_front"].append(image_from_msg(
                    history_row["camera_front"], cfg["front_image_width"], cfg["front_image_height"]
                ))
                for side in ("camera_left", "camera_right"):
                    sample[side].append(image_from_msg(
                        history_row[side], cfg["side_image_width"], cfg["side_image_height"]
                    ))
                xyz = pointcloud2_to_xyz(history_row["lidar"])
                sample["lidar"].append(points_to_bev(
                    xyz, cfg["bev_x_range"], cfg["bev_y_range"],
                    cfg["bev_height"], cfg["bev_width"],
                ))
                sample["ego"].append(ego_localization_vector(
                    history_row["reference"], history_row["imu"],
                    history_row["vehicle_status"], history_row["gps"],
                    history_row["stamp"], history_row["local_route"], velocity_scale,
                ))
            sample["map_tokens"], sample["map_mask"] = mgeo_tokens_from_msg(
                row["mgeo_token"], cfg["max_map_tokens"], cfg["map_token_dim"], True
            )
            sample["route_tokens"], sample["route_mask"] = route_tokens_from_msg(
                row["local_route"], cfg["max_route_tokens"],
                cfg["route_normalization_m"], cfg.get("local_route_frame", "base_link"), True,
            )
        except (ValueError, TypeError) as error:
            raise RuntimeError("V2 input validation failed at {:.3f}: {}".format(row["stamp"], error))

        arrays = {key: np.asarray(value) for key, value in sample.items()}
        arrays["trajectory"] = label
        arrays.update(auxiliary_labels(label))
        arrays["stamp"] = np.asarray(row["stamp"], np.float64)
        prefix = args.prefix or Path(args.bag).stem
        prefix = re.sub(r"[^A-Za-z0-9_.-]+", "_", prefix).strip("_") or "bag"
        np.savez_compressed(
            output / "{}_sample_{:08d}.npz".format(prefix, written), **arrays
        )
        written += 1
        last_sample_stamp = row["stamp"]
        if args.max_samples is not None and written >= args.max_samples:
            break

    if written == 0:
        raise RuntimeError("Bag is too short for 1s history + 4s prediction, or has no valid samples")
    print("Wrote {} V2 samples to {}".format(written, output))


if __name__ == "__main__":
    main()
