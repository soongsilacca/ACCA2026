#!/usr/bin/env python3
import csv
import json
import math
import os

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from scipy.spatial import cKDTree
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class MGeoRoutePublisher:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/kinematic_state")
        self.global_path_csv = os.path.expanduser(
            rospy.get_param("~global_path_csv", "~/acca_ws/global_path/global_path.csv")
        )
        self.global_path_topic = rospy.get_param("~global_path_topic", "/global_path")
        self.local_route_topic = rospy.get_param("~local_route_topic", "/local_route")
        self.token_topic = rospy.get_param("~mgeo_token_topic", "/mgeo_tokens")
        self.ego_frame = rospy.get_param("~ego_frame", "base_link")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.publish_hz = float(rospy.get_param("~publish_hz", 10.0))
        self.route_points = int(rospy.get_param("~route_points", 32))
        self.lookbehind = int(rospy.get_param("~route_lookbehind_points", 2))
        self.route_stride = max(1, int(rospy.get_param("~route_sample_stride", 5)))
        self.max_match_distance = float(rospy.get_param("~max_path_match_distance", 20.0))
        self.radius = float(rospy.get_param("~map_radius", 60.0))
        self.max_tokens = int(rospy.get_param("~max_map_tokens", 128))
        self.sample_spacing = float(rospy.get_param("~link_sample_spacing", 3.0))
        self.speed_topic = rospy.get_param(
            "~mgeo_target_velocity_topic", "/mgeo_target_velocity"
        )
        self.speed_cap_mps = float(
            rospy.get_param("~speed_cap_kph", 100.0)
        ) / 3.6
        self.normal_road_speed_mps = float(
            rospy.get_param("~normal_road_speed_kph", 60.0)
        ) / 3.6
        self.highway_speed_mps = float(
            rospy.get_param("~highway_speed_kph", 100.0)
        ) / 3.6
        self.default_speed_mps = float(
            rospy.get_param("~default_speed_kph", 60.0)
        ) / 3.6
        self.max_lateral_accel = float(
            rospy.get_param("~max_lateral_accel", 1.8)
        )
        self.min_curve_speed_mps = float(
            rospy.get_param("~min_curve_speed_kph", 20.0)
        ) / 3.6
        self.max_longitudinal_decel = float(
            rospy.get_param("~max_longitudinal_decel", 3.0)
        )
        self.target_speed_decel_rate = float(
            rospy.get_param("~target_speed_decel_rate", 1.5)
        )
        self.speed_reaction_time = float(
            rospy.get_param("~speed_reaction_time", 0.5)
        )
        self.speed_lookahead_m = float(
            rospy.get_param("~speed_lookahead_m", 20.0)
        )
        self.speed_filter_alpha = float(
            rospy.get_param("~speed_filter_alpha", 0.8)
        )

        self.map_points, self.map_attributes = self.load_mgeo(
            rospy.get_param("~link_file")
        )
        self.ego_pose = None
        self.global_path = self.load_global_path_csv(self.global_path_csv)
        self.global_path_s = self.path_distance(self.global_path)
        self.global_path_speed = self.build_speed_profile()
        self.filtered_target_speed = None
        self.ego_speed = 0.0
        self.last_path_index = 0

        self.route_pub = rospy.Publisher(self.local_route_topic, Path, queue_size=1)
        self.token_pub = rospy.Publisher(self.token_topic, Float32MultiArray, queue_size=1)
        self.global_path_pub = rospy.Publisher(self.global_path_topic, Path, queue_size=1, latch=True)
        self.speed_pub = rospy.Publisher(self.speed_topic, Float32, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_hz, 0.1)), self.timer_callback)
        self.publish_global_path()
        rospy.loginfo(
            "MGeo route publisher ready: samples=%d path_points=%d route=%s tokens=%s",
            len(self.map_points), len(self.global_path), self.local_route_topic, self.token_topic,
        )

    def publish_global_path(self):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        for index, (x, y) in enumerate(self.global_path):
            next_index = (index + 1) % len(self.global_path)
            delta = self.global_path[next_index] - self.global_path[index]
            yaw = math.atan2(float(delta[1]), float(delta[0]))
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            msg.poses.append(pose)
        self.global_path_pub.publish(msg)

    @staticmethod
    def load_global_path_csv(csv_path):
        points = []
        with open(csv_path, "r", newline="") as stream:
            reader = csv.DictReader(stream)
            if not reader.fieldnames or "x" not in reader.fieldnames or "y" not in reader.fieldnames:
                raise ValueError("global path CSV must contain x and y columns")
            for row_number, row in enumerate(reader, start=2):
                try:
                    points.append([float(row["x"]), float(row["y"])])
                except (TypeError, ValueError):
                    rospy.logwarn("Skipping invalid global path CSV row %d", row_number)
        if len(points) < 2:
            raise ValueError("global path CSV needs at least two valid points")
        return np.asarray(points, dtype=np.float32)

    def load_mgeo(self, link_file):
        with open(link_file, "r") as stream:
            links = json.load(stream)
        points, attributes = [], []
        for link in links:
            raw = np.asarray(link.get("points", []), dtype=np.float32)
            if len(raw) < 2:
                continue
            try:
                lane_type = float(link.get("link_type", 0)) / 10.0
            except (TypeError, ValueError):
                lane_type = 0.0
            try:
                max_speed_kph = float(link.get("max_speed", 0))
                speed_zone = max_speed_kph > 60.0
            except (TypeError, ValueError):
                max_speed_kph = 0.0
                speed_zone = False
            width_start = float(link.get("width_start", 3.5) or 3.5)
            width_end = float(link.get("width_end", width_start) or width_start)
            last_point = None
            for index in range(len(raw) - 1):
                point = raw[index, :2]
                if last_point is not None and np.linalg.norm(point - last_point) < self.sample_spacing:
                    continue
                before = raw[max(index - 1, 0), :2]
                after = raw[min(index + 1, len(raw) - 1), :2]
                tangent = after - before
                tangent_norm = np.linalg.norm(tangent)
                if tangent_norm < 1e-5:
                    continue
                tangent /= tangent_norm
                curvature = self.curvature(raw, index)
                ratio = index / max(len(raw) - 1, 1)
                half_width = 0.5 * ((1.0 - ratio) * width_start + ratio * width_end)
                points.append(point)
                attributes.append([
                    tangent[0], tangent[1], curvature,
                    half_width, lane_type, float(speed_zone),
                    max_speed_kph / 3.6,
                ])
                last_point = point
        return np.asarray(points, dtype=np.float32), np.asarray(attributes, dtype=np.float32)

    @staticmethod
    def curvature(points, index):
        i0, i2 = max(0, index - 1), min(len(points) - 1, index + 1)
        if i0 == index or i2 == index:
            return 0.0
        first = points[index, :2] - points[i0, :2]
        second = points[i2, :2] - points[index, :2]
        lengths = (np.linalg.norm(first), np.linalg.norm(second))
        if min(lengths) < 1e-4:
            return 0.0
        cross = first[0] * second[1] - first[1] * second[0]
        angle = math.atan2(float(cross), float(np.dot(first, second)))
        return angle / max(0.5 * sum(lengths), 1e-4)

    @staticmethod
    def path_distance(points):
        return np.concatenate((
            [0.0],
            np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1)),
        ))

    def build_speed_profile(self):
        """Match the global path to MGeo speed limits and curvature limits."""
        tree = cKDTree(self.map_points)
        _distance, nearest = tree.query(self.global_path, k=1)
        highway = self.map_attributes[nearest, 5] > 0.5
        link_speed = np.where(
            highway,
            self.highway_speed_mps,
            self.normal_road_speed_mps,
        )

        dx = np.gradient(self.global_path[:, 0])
        dy = np.gradient(self.global_path[:, 1])
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        curvature = np.abs(
            (dx * ddy - dy * ddx)
            / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-8)
        )
        curve_speed = np.sqrt(
            self.max_lateral_accel / np.maximum(curvature, 1e-4)
        )
        curve_speed = np.maximum(curve_speed, self.min_curve_speed_mps)
        profile = np.minimum.reduce((
            link_speed,
            curve_speed,
            np.full_like(link_speed, self.speed_cap_mps),
        ))

        # A curve speed limit at the curve itself is too late to brake for it.
        # Propagate each limit backwards using v0^2 <= v1^2 + 2*a*ds so the
        # target speed already starts falling on the preceding straight.
        max_decel = max(self.max_longitudinal_decel, 0.1)
        segment_length = np.linalg.norm(
            np.diff(self.global_path, axis=0), axis=1
        )
        for _pass in range(2):
            for index in range(len(profile) - 2, -1, -1):
                reachable_speed = math.sqrt(
                    max(
                        profile[index + 1] ** 2
                        + 2.0 * max_decel * segment_length[index],
                        0.0,
                    )
                )
                profile[index] = min(profile[index], reachable_speed)

        rospy.loginfo(
            "MGeo speed profile ready: %.1f-%.1f km/h, %d points",
            float(np.min(profile) * 3.6),
            float(np.max(profile) * 3.6),
            len(profile),
        )
        return profile.astype(np.float32)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.ego_pose = np.asarray(
            [position.x, position.y, yaw_from_quaternion(msg.pose.pose.orientation)],
            dtype=np.float32,
        )
        velocity = msg.twist.twist.linear
        self.ego_speed = math.hypot(float(velocity.x), float(velocity.y))

    def world_to_ego(self, points):
        delta = points - self.ego_pose[:2]
        c, s = math.cos(float(self.ego_pose[2])), math.sin(float(self.ego_pose[2]))
        return delta @ np.asarray([[c, -s], [s, c]], dtype=np.float32)

    def timer_callback(self, _event):
        if self.ego_pose is None:
            return
        stamp = rospy.Time.now()
        self.publish_tokens(stamp)
        self.publish_local_route(stamp)
        self.publish_target_speed()

    def publish_tokens(self, _stamp):
        local = self.world_to_ego(self.map_points)
        distance = np.linalg.norm(local, axis=1)
        candidates = np.flatnonzero(distance <= self.radius)
        candidates = candidates[np.argsort(distance[candidates])[:self.max_tokens]]
        token = np.zeros((self.max_tokens, 8), dtype=np.float32)
        c, s = math.cos(float(self.ego_pose[2])), math.sin(float(self.ego_pose[2]))
        tangent_rotation = np.asarray([[c, -s], [s, c]], dtype=np.float32)
        tangent_local = self.map_attributes[candidates, :2] @ tangent_rotation
        for row, index in enumerate(candidates):
            token[row] = [
                local[index, 0] / self.radius,
                local[index, 1] / self.radius,
                tangent_local[row, 0], tangent_local[row, 1],
                np.clip(self.map_attributes[index, 2] * 10.0, -1.0, 1.0),
                np.clip(self.map_attributes[index, 3] / self.radius, 0.0, 1.0),
                np.clip(self.map_attributes[index, 4], 0.0, 1.0),
                self.map_attributes[index, 5],
            ]
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label="mgeo_v2_points", size=self.max_tokens, stride=self.max_tokens * 8),
            MultiArrayDimension(label="features", size=8, stride=8),
        ]
        msg.data = token.reshape(-1).tolist()
        self.token_pub.publish(msg)

    def publish_local_route(self, stamp):
        distance = np.linalg.norm(self.global_path - self.ego_pose[:2], axis=1)
        closest = int(np.argmin(distance))
        if distance[closest] > self.max_match_distance:
            rospy.logwarn_throttle(2.0, "ego is %.2fm from global_path", distance[closest])
        self.last_path_index = closest
        # The CSV is a closed loop. Wrap local route indices at the seam.
        unique_count = len(self.global_path)
        if unique_count > 1 and np.linalg.norm(self.global_path[0] - self.global_path[-1]) < 1e-4:
            unique_count -= 1
        start = (closest - self.lookbehind * self.route_stride) % unique_count
        indices = (start + np.arange(self.route_points) * self.route_stride) % unique_count
        selected = self.global_path[indices]
        local = self.world_to_ego(selected)
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = self.ego_frame
        for x, y in local:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.route_pub.publish(msg)

    def publish_target_speed(self):
        if len(self.global_path_speed) == 0:
            return
        start_s = self.global_path_s[self.last_path_index]
        braking_distance = (
            self.ego_speed ** 2
            / (2.0 * max(self.max_longitudinal_decel, 0.1))
            + self.ego_speed * max(self.speed_reaction_time, 0.0)
        )
        lookahead_distance = max(self.speed_lookahead_m, braking_distance)
        end_s = start_s + lookahead_distance
        end_index = int(np.searchsorted(
            self.global_path_s, end_s, side="right"
        ))
        if end_index <= self.last_path_index:
            end_index = self.last_path_index + 1
        end_index = min(end_index, len(self.global_path_speed))
        target = float(np.min(
            self.global_path_speed[self.last_path_index:end_index]
        ))
        if self.filtered_target_speed is None:
            self.filtered_target_speed = target
        elif target < self.filtered_target_speed:
            # Do not step the target speed down in one 10 Hz cycle: that makes
            # the MPC request emergency-like braking. The preview profile gives
            # enough distance to lower it with a bounded deceleration ramp.
            max_speed_drop = (
                max(self.target_speed_decel_rate, 0.1)
                / max(self.publish_hz, 0.1)
            )
            self.filtered_target_speed = max(
                target,
                self.filtered_target_speed - max_speed_drop,
            )
        else:
            # Keep acceleration smooth after leaving the curve.
            alpha = np.clip(self.speed_filter_alpha, 0.0, 1.0)
            self.filtered_target_speed = (
                alpha * self.filtered_target_speed
                + (1.0 - alpha) * target
            )
        self.speed_pub.publish(Float32(data=self.filtered_target_speed))
        rospy.loginfo_throttle(
            1.0,
            "MGeo target speed: %.2f m/s (%.1f km/h, preview %.1fm)",
            self.filtered_target_speed,
            self.filtered_target_speed * 3.6,
            lookahead_distance,
        )


if __name__ == "__main__":
    rospy.init_node("mgeo_route_publisher")
    try:
        MGeoRoutePublisher()
        rospy.spin()
    except (rospy.ROSInterruptException, IOError, ValueError) as error:
        rospy.logfatal("mgeo_route_publisher failed: %s", error)
