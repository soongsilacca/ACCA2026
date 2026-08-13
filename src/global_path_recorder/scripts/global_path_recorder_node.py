#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, Int32


class GlobalPathRecorderNode:
    def __init__(self):
        rospy.init_node("global_path_recorder_node")

        self.odom_topic = rospy.get_param("~odom_topic", "/localization/kinematic_state")
        self.path_topic = rospy.get_param("~path_topic", "/global_path")
        self.csv_path = os.path.expanduser(
            rospy.get_param("~csv_path", "/tmp/global_path.csv")
        )
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.sample_distance = float(rospy.get_param("~sample_distance", 0.1))
        self.publish_hz = float(rospy.get_param("~publish_hz", 10.0))
        self.write_csv = bool(rospy.get_param("~write_csv", True))
        self.default_token = int(rospy.get_param("~default_token", 0))
        self.csv_rewrite_every_points = int(
            rospy.get_param("~csv_rewrite_every_points", 20)
        )

        self.sample_distance = max(self.sample_distance, 0.001)
        self.csv_rewrite_every_points = max(self.csv_rewrite_every_points, 1)
        self.points = []
        self.last_point = None
        self.total_length = 0.0

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.count_pub = rospy.Publisher("~point_count", Int32, queue_size=1)
        self.length_pub = rospy.Publisher("~path_length", Float32, queue_size=1)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_hz, 0.1)),
            self.timer_callback,
        )
        rospy.on_shutdown(self.write_csv_file)

        rospy.loginfo(
            "global_path_recorder_node ready. odom=%s path=%s csv=%s sample=%.3fm token=%d",
            self.odom_topic,
            self.path_topic,
            self.csv_path if self.write_csv else "disabled",
            self.sample_distance,
            self.default_token,
        )

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.yaw_from_odom(msg)

        if self.last_point is None:
            self.add_point(msg.header.stamp, x, y, yaw, 0.0)
            return

        last_x, last_y, _last_yaw = self.last_point
        distance = math.hypot(x - last_x, y - last_y)
        if distance < self.sample_distance:
            return

        self.total_length += distance
        self.add_point(msg.header.stamp, x, y, yaw, self.total_length)

    def add_point(self, stamp, x, y, yaw, s):
        self.last_point = (x, y, yaw)
        self.points.append((stamp, x, y, yaw, s))

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.path_msg.poses.append(pose)

        if self.write_csv and len(self.points) % self.csv_rewrite_every_points == 0:
            self.write_csv_file()

    def timer_callback(self, _event):
        now = rospy.Time.now()
        self.path_msg.header.stamp = now
        self.path_pub.publish(self.path_msg)
        self.count_pub.publish(Int32(data=len(self.points)))
        self.length_pub.publish(Float32(data=self.total_length))

    @staticmethod
    def yaw_from_odom(msg):
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        return tf.transformations.euler_from_quaternion(quat)[2]

    def write_csv_file(self):
        if not self.write_csv or not self.csv_path:
            return

        directory = os.path.dirname(self.csv_path)
        if directory:
            os.makedirs(directory, exist_ok=True)

        tmp_path = self.csv_path + ".tmp"
        with open(tmp_path, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["stamp", "x", "y", "yaw", "s", "token"])

            for stamp, x, y, yaw, s in self.points:
                writer.writerow([stamp.to_sec(), x, y, yaw, s, self.default_token])

        os.replace(tmp_path, self.csv_path)


if __name__ == "__main__":
    try:
        GlobalPathRecorderNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
