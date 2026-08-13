import numpy as np


def pointcloud2_to_xyz(msg):
    from sensor_msgs import point_cloud2
    return np.asarray(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32)


def points_to_bev(points, x_range, y_range, height, width):
    """Three channels: occupancy, normalized maximum height and log density."""
    bev = np.zeros((3, height, width), dtype=np.float32)
    if points.size == 0:
        return bev
    valid = ((points[:, 0] >= x_range[0]) & (points[:, 0] < x_range[1]) &
             (points[:, 1] >= y_range[0]) & (points[:, 1] < y_range[1]))
    p = points[valid]
    if not len(p):
        return bev
    rows = np.clip(((x_range[1] - p[:, 0]) / (x_range[1] - x_range[0]) * height).astype(int), 0, height - 1)
    cols = np.clip(((y_range[1] - p[:, 1]) / (y_range[1] - y_range[0]) * width).astype(int), 0, width - 1)
    np.add.at(bev[2], (rows, cols), 1.0)
    np.maximum.at(bev[1], (rows, cols), np.clip((p[:, 2] + 3.0) / 6.0, 0.0, 1.0))
    bev[0, rows, cols] = 1.0
    bev[2] = np.log1p(bev[2]) / np.log(64.0)
    return np.clip(bev, 0.0, 1.0)
