import math
import numpy as np


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_from_odometry(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return np.asarray([p.x, p.y, yaw_from_quaternion(q)], dtype=np.float32)


def world_to_ego(points, pose):
    points = np.asarray(points, dtype=np.float32)
    delta = points[..., :2] - pose[:2]
    c, s = math.cos(float(pose[2])), math.sin(float(pose[2]))
    rotation = np.asarray([[c, s], [-s, c]], dtype=np.float32)
    return delta @ rotation.T


def ego_to_world(points, pose):
    points = np.asarray(points, dtype=np.float32)
    c, s = math.cos(float(pose[2])), math.sin(float(pose[2]))
    rotation = np.asarray([[c, -s], [s, c]], dtype=np.float32)
    return points @ rotation.T + pose[:2]
