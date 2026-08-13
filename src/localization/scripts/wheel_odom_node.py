#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wheel_odom_node.py — MORAI Wheel Odometry Node (ROS1)

Subscribes to /imu and /Competition_topic (EgoVehicleStatus).
Publishes /wheel_odometry (Odometry, odom frame).

  /imu                ─┐
                        ├─► WheelOdomNode ──► /wheel_odometry
  /Competition_topic  ─┘
"""

import math
import rospy
from sensor_msgs.msg import Imu
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ── Constants ────────────────────────────────────────────────────────────────
MAX_DT         = 1.0     # Ignore dt larger than this [s]
SPEED_DEADBAND = 0.05    # Below this speed, treat as stopped [m/s]

# Covariance values (ref: basic_localization)
COV_POSE_XY   = 0.01
COV_POSE_YAW  = 0.05
COV_TWIST_VX  = 0.01
COV_TWIST_VY  = 0.001   # No-slip constraint
COV_TWIST_YAW = 0.05
COV_FILL      = 1e-6    # Off-diagonal / unused diagonal fill


class WheelOdomNode:
    def __init__(self):
        rospy.init_node('wheel_odom_node', anonymous=True)

        # Vehicle parameters
        self.wheelbase = rospy.get_param('~vehicle/wheelbase', 3.000)

        # State
        self.is_initialized = False
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

        # Subscribers
        self.imu_sub    = rospy.Subscriber('/imu',              Imu,             self.imu_callback)
        self.status_sub = rospy.Subscriber('/morai/ego_vehicle_status', EgoVehicleStatus, self.status_callback)

        # Publisher
        self.odom_pub = rospy.Publisher('/wheel_odometry', Odometry, queue_size=10)

        rospy.loginfo(f"WheelOdomNode ready (wheelbase={self.wheelbase}m). Waiting for IMU...")

    # ── IMU callback: one-time heading init ──────────────────────────────────

    def imu_callback(self, msg):
        if self.is_initialized:
            return

        stamp = msg.header.stamp
        if stamp.to_sec() == 0:
            stamp = rospy.Time.now()

        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.theta = yaw
        self.is_initialized = True
        rospy.loginfo(f"IMU heading initialized: {math.degrees(yaw):.1f} deg ({yaw:.4f} rad)")

        # Publish initial stationary odometry so EKF has data immediately
        self._publish(rospy.Time.now(), 0.0, 0.0)

    # ── Vehicle status callback ───────────────────────────────────────────────

    def status_callback(self, msg):
        if not self.is_initialized:
            return

        stamp = msg.header.stamp
        if stamp.to_sec() == 0:
            stamp = rospy.Time.now()

        if self.last_time is None:
            self.last_time = stamp
            return

        dt = (stamp - self.last_time).to_sec()
        self.last_time = stamp

        if dt <= 0 or dt > MAX_DT:
            return

        # signed_vel is the vehicle's longitudinal speed (body frame) in km/h — convert to m/s
        speed = msg.signed_vel / 3.6
        v = speed if abs(speed) > SPEED_DEADBAND else 0.0

        # angular_velocity.z is the vehicle's yaw rate in deg/s — convert to rad/s (ROS CCW positive)
        # Note: Invert sign because simulator yaw rate convention is opposite to ROS
        omega = -math.radians(msg.angular_velocity.z)

        self.x     += v * math.cos(self.theta) * dt
        self.y     += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self._publish(stamp, v, omega)

    # ── Odometry publisher ────────────────────────────────────────────────────

    def _publish(self, stamp, v: float, omega: float):
        q = quaternion_from_euler(0, 0, self.theta)

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation   = Quaternion(*q)
        odom.pose.covariance         = _make_cov36(COV_POSE_XY, COV_POSE_XY, COV_POSE_YAW)

        odom.twist.twist.linear.x  = v
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.angular.z = omega
        odom.twist.covariance      = _make_cov36(COV_TWIST_VX, COV_TWIST_VY, COV_TWIST_YAW)

        self.odom_pub.publish(odom)


# ── Utility ───────────────────────────────────────────────────────────────────

def _make_cov36(d0: float, d7: float, d35: float) -> list:
    """Build a 6×6 covariance (row-major).
    Used dimensions (x, y, yaw for pose / vx, vy, vyaw for twist) get proper values.
    Unused diagonal entries (z=14, roll=21, pitch=28) get large values so EKF ignores them.
    Off-diagonal entries are 0 (no cross-correlations assumed).
    """
    cov = [0.0] * 36
    cov[0]  = d0
    cov[7]  = d7
    cov[14] = 999.0   # z  — ignored
    cov[21] = 999.0   # roll — ignored
    cov[28] = 999.0   # pitch — ignored
    cov[35] = d35
    return cov


if __name__ == '__main__':
    try:
        node = WheelOdomNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
