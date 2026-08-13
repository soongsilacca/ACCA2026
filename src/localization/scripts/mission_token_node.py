#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, Int32


class MissionTokenNode:
    def __init__(self):
        rospy.init_node("mission_token_node")

        self.global_path_topic = rospy.get_param("~global_path_topic", "/global_path")
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/kinematic_state")
        self.token_topic = rospy.get_param("~token_topic", "/mission_token")
        self.segment_index_topic = rospy.get_param(
            "~segment_index_topic",
            "/mission_token/segment_index",
        )
        self.progress_topic = rospy.get_param("~progress_topic", "/mission_token/progress")
        self.publish_hz = rospy.get_param("~publish_hz", 20.0)

        self.num_segments = int(rospy.get_param("~num_segments", 10))
        if self.num_segments < 1:
            rospy.logwarn("~num_segments must be >= 1. Using 1.")
            self.num_segments = 1

        self.token_values = rospy.get_param("~token_values", [])
        self.default_token_offset = int(rospy.get_param("~default_token_offset", 0))
        self.max_match_distance = float(rospy.get_param("~max_match_distance", 15.0))

        self.path_points = []
        self.path_s = []
        self.total_s = 0.0
        self.latest_odom = None
        self.last_closest_idx = 0

        self.pub_token = rospy.Publisher(self.token_topic, Int32, queue_size=10)
        self.pub_segment_index = rospy.Publisher(
            self.segment_index_topic,
            Int32,
            queue_size=10,
        )
        self.pub_progress = rospy.Publisher(self.progress_topic, Float32, queue_size=10)

        rospy.Subscriber(self.global_path_topic, Path, self.path_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=20)

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_hz, 0.1)),
            self.timer_callback,
        )

        rospy.loginfo(
            "mission_token_node ready. path=%s odom=%s segments=%d",
            self.global_path_topic,
            self.odom_topic,
            self.num_segments,
        )

    def path_callback(self, msg):
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(points) < 2:
            rospy.logwarn_throttle(5.0, "Global path needs at least 2 poses.")
            return

        path_s = [0.0]
        for i in range(1, len(points)):
            px, py = points[i - 1]
            x, y = points[i]
            path_s.append(path_s[-1] + math.hypot(x - px, y - py))

        if path_s[-1] <= 0.0:
            rospy.logwarn_throttle(5.0, "Global path length is zero.")
            return

        self.path_points = points
        self.path_s = path_s
        self.total_s = path_s[-1]
        self.last_closest_idx = min(self.last_closest_idx, len(points) - 1)

        rospy.loginfo(
            "mission_token_node loaded path. points=%d length=%.2fm",
            len(points),
            self.total_s,
        )

    def odom_callback(self, msg):
        self.latest_odom = msg

    def timer_callback(self, _event):
        if self.latest_odom is None or not self.path_points:
            return

        x = self.latest_odom.pose.pose.position.x
        y = self.latest_odom.pose.pose.position.y

        closest_idx, closest_dist = self.find_closest_index(x, y)
        if closest_idx is None:
            return

        if closest_dist > self.max_match_distance:
            rospy.logwarn_throttle(
                2.0,
                "Localization is %.2fm away from global_path. Keeping nearest token.",
                closest_dist,
            )

        self.last_closest_idx = closest_idx
        progress = self.path_s[closest_idx] / self.total_s
        progress = min(max(progress, 0.0), 1.0)

        segment_index = int(progress * self.num_segments)
        if segment_index >= self.num_segments:
            segment_index = self.num_segments - 1

        token = self.token_for_segment(segment_index)

        self.pub_token.publish(Int32(data=token))
        self.pub_segment_index.publish(Int32(data=segment_index))
        self.pub_progress.publish(Float32(data=progress))

    def find_closest_index(self, x, y):
        if not self.path_points:
            return None, float("inf")

        # Search around the previous match first. This is stable for forward driving.
        window = int(rospy.get_param("~search_window", 80))
        start = max(0, self.last_closest_idx - window)
        end = min(len(self.path_points), self.last_closest_idx + window + 1)

        idx, dist = self.closest_in_range(x, y, start, end)

        # If the vehicle spawned far away or path jumped, fall back to full scan.
        if dist > self.max_match_distance:
            idx, dist = self.closest_in_range(x, y, 0, len(self.path_points))

        return idx, dist

    def closest_in_range(self, x, y, start, end):
        best_idx = None
        best_dist_sq = float("inf")

        for i in range(start, end):
            px, py = self.path_points[i]
            dist_sq = (x - px) * (x - px) + (y - py) * (y - py)
            if dist_sq < best_dist_sq:
                best_idx = i
                best_dist_sq = dist_sq

        return best_idx, math.sqrt(best_dist_sq)

    def token_for_segment(self, segment_index):
        if self.token_values:
            if segment_index < len(self.token_values):
                return int(self.token_values[segment_index])
            return int(self.token_values[-1])

        return self.default_token_offset + segment_index


if __name__ == "__main__":
    try:
        MissionTokenNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
