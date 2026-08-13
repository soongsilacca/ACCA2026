import numpy as np

from .coordinate_transform import world_to_ego
from .synchronizer import nearest_index


def future_trajectory(poses, speeds, stamps, current_index, horizon, interval=0.2, tolerance=0.08):
    """Return V2 absolute ego-local [x,y,relative_yaw,target_speed] labels."""
    current_stamp = stamps[current_index]
    indices = [
        nearest_index(stamps, current_stamp + (step + 1) * interval, tolerance)
        for step in range(horizon)
    ]
    if any(index is None or index <= current_index for index in indices):
        return None
    indices = np.asarray(indices, dtype=np.int64)
    current_pose = poses[current_index]
    local_points = world_to_ego(np.asarray(poses)[indices, :2], current_pose)
    relative_yaw = poses[indices, 2] - current_pose[2]
    relative_yaw = np.arctan2(np.sin(relative_yaw), np.cos(relative_yaw))
    return np.column_stack((local_points, relative_yaw, speeds[indices])).astype(np.float32)


def auxiliary_labels(trajectory):
    speed = trajectory[:, 3]
    stopped = bool(np.any(speed < 0.2))
    stop_distance = 0.0
    if stopped:
        index = int(np.flatnonzero(speed < 0.2)[0])
        stop_distance = float(np.linalg.norm(trajectory[index, :2]))
    return {
        "signal_label": np.asarray(-1, dtype=np.int64),
        "stop_label": np.asarray(float(stopped), dtype=np.float32),
        "stop_distance_label": np.asarray(stop_distance, dtype=np.float32),
        "hazard_label": np.asarray(-1.0, dtype=np.float32),
        "ttc_label": np.asarray(-1, dtype=np.int64),
        "lane_label": np.asarray(-1.0, dtype=np.float32),
    }
