#!/usr/bin/env python3
"""MORAI EgoVehicleStatus UDP -> exact teacher status and ground-truth odometry."""
import math
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from lib.network.UDP import Receiver
from lib.define.EgoVehicleStatus import EgoVehicleStatus as EgoVehicleStatusDef


def build_odometry(raw, stamp, frame_id, child_frame_id):
    msg = Odometry()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.child_frame_id = child_frame_id
    msg.pose.pose.position.x = float(raw.pos_x)
    msg.pose.pose.position.y = float(raw.pos_y)
    # The planning map/localization in this workspace is intentionally 2-D.
    msg.pose.pose.position.z = 0.0
    yaw = math.radians(float(raw.yaw))
    q = quaternion_from_euler(0.0, 0.0, yaw)
    msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = q[0], q[1]
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q[2], q[3]

    # MORAI EgoVehicleStatus linear velocity is km/h and body-frame based.
    msg.twist.twist.linear.x = float(raw.vel_x) / 3.6
    msg.twist.twist.linear.y = float(raw.vel_y) / 3.6
    msg.twist.twist.linear.z = float(raw.vel_z) / 3.6
    msg.twist.twist.angular.x = math.radians(float(raw.ang_vel_x))
    msg.twist.twist.angular.y = math.radians(float(raw.ang_vel_y))
    msg.twist.twist.angular.z = math.radians(float(raw.ang_vel_z))
    # Mark raw simulator pose as a high-confidence teacher signal.
    msg.pose.covariance[0] = 1e-6
    msg.pose.covariance[7] = 1e-6
    msg.pose.covariance[35] = 1e-6
    msg.twist.covariance[0] = 1e-4
    msg.twist.covariance[7] = 1e-4
    msg.twist.covariance[35] = 1e-4
    return msg


def main():
    rospy.init_node("morai_ego_ground_truth_state")
    ip = rospy.get_param("~ip", "127.0.0.1")
    port = int(rospy.get_param("~port", 9011))
    gt_topic = rospy.get_param("~gt_topic", "/teacher/ground_truth_state")
    frame_id = rospy.get_param("~frame_id", "map")
    child_frame_id = rospy.get_param("~child_frame_id", "base_link")
    gt_pub = rospy.Publisher(gt_topic, Odometry, queue_size=1)
    receiver = Receiver(ip, port, EgoVehicleStatusDef())
    rospy.loginfo("[TeacherGroundTruth] MORAI UDP %s:%d raw.pos_x/y/yaw -> %s",
                  ip, port, gt_topic)
    try:
        while not rospy.is_shutdown():
            try:
                raw = receiver._queue.get(timeout=0.5)
            except Exception:
                rospy.logwarn_throttle(5.0, "[TeacherGroundTruth] waiting for UDP %s:%d", ip, port)
                continue
            if int(raw.sec) == 0:
                continue
            stamp = rospy.Time.now()
            gt_pub.publish(build_odometry(raw, stamp, frame_id, child_frame_id))
    finally:
        receiver.stop()


if __name__ == "__main__":
    main()
