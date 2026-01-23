#!/usr/bin/env python3
"""
FAST-LIO Odometry Adapter

Converts FAST-LIO odometry (camera_init -> body) to align with localization EKF (odom -> base_link)
Automatically corrects initial orientation mismatch.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf_transformations
import numpy as np
import math


class FastLIOOdometryAdapter(Node):
    def __init__(self):
        super().__init__("fastlio_odometry_adapter_node")
        
        # Parameters
        self.declare_parameter("use_twist", False)
        self.use_twist = self.get_parameter("use_twist").value
        
        # State
        self.yaw_offset = None
        self.offset_initialized = False
        self.ekf_yaw = None
        self.fastlio_first_msg = None
        self.last_fastlio_yaw = None
        
        # Subscribers
        self.sub_fastlio = self.create_subscription(
            Odometry, "/Odometry", self.fastlio_callback, 10
        )
        self.sub_ekf = self.create_subscription(
            Odometry, "/odometry/local", self.ekf_callback, 10
        )
        self.sub_imu = self.create_subscription(
            Imu, "/imu/data_nwu", self.imu_callback, 10
        )
        
        self.pub_aligned = self.create_publisher(Odometry, "/fastlio/odom_aligned", 10)
        
        self.get_logger().info("FAST-LIO Odometry Adapter started")
        self.get_logger().info("Waiting for initial messages (IMU or EKF) to calculate yaw offset...")
        
    def imu_callback(self, msg: Imu):
        """Use IMU for initial initialization (breaks deadlock)"""
        if self.offset_initialized:
            return  # Only use IMU for first init
            
        orientation = msg.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        # Consider this as EKF yaw estimate for initialization
        self.ekf_yaw = yaw
        
        if not self.offset_initialized:
            self.get_logger().info(f"IMU initial yaw: {math.degrees(yaw):.2f}°")
            self.try_initialize()
        
    
    def ekf_callback(self, msg: Odometry):
        """Store EKF yaw for offset calculation and drift check"""
        # Extract yaw from EKF
        orientation = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        self.ekf_yaw = yaw
        
        # Initial initialization
        if not self.offset_initialized:
            self.get_logger().info(f"EKF initial yaw: {math.degrees(yaw):.2f}°")
            self.try_initialize()
            return

        # Check for Re-initialization (e.g. if EKF got a set_pose and jumped)
        # Compare EKF current yaw with what we just published (FAST-LIO + Offset)
        # We need the LAST processed fastlio quaternion to assume what "aligned" yaw is currently
        if self.last_fastlio_yaw is not None:
            current_aligned_yaw = self.normalize_angle(self.last_fastlio_yaw + self.yaw_offset)
            diff = abs(self.normalize_angle(yaw - current_aligned_yaw))
            
            # If difference > 20 degrees (0.35 rad), assume EKF reset happened
            if diff > 0.35: 
                self.get_logger().warn(f"Large Yaw difference detected detected ({math.degrees(diff):.2f}°)! Re-aligning adapter...")
                self.offset_initialized = False
                # try_initialize will be called on next loop or recursively
                self.try_initialize()

    def fastlio_callback(self, msg: Odometry):
        """Process FAST-LIO odometry"""
        # Extract current FAST-LIO yaw
        orientation = msg.pose.pose.orientation
        _, _, yaw_f = tf_transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.last_fastlio_yaw = yaw_f
        
        # Store first FAST-LIO message for initialization
        if self.fastlio_first_msg is None:
            self.fastlio_first_msg = msg
            self.get_logger().info(f"FAST-LIO initial yaw: {math.degrees(yaw_f):.2f}°")
            self.try_initialize()
        
        # Wait for initialization
        if not self.offset_initialized:
            return
        
        # Transform and publish
        aligned_msg = self.transform_odometry(msg, yaw_f)
        self.pub_aligned.publish(aligned_msg)
    
    def try_initialize(self):
        """Try to initialize yaw offset if both messages received"""
        if self.offset_initialized:
            return
        
        if self.ekf_yaw is None or self.last_fastlio_yaw is None:
            return
        
        # Calculate offset based on CURRENT values, not old snapshots
        self.yaw_offset = self.ekf_yaw - self.last_fastlio_yaw
        self.offset_initialized = True
        
        self.get_logger().info(
            f"✓ Yaw offset initialized: {math.degrees(self.yaw_offset):.2f}° "
            f"(EKF: {math.degrees(self.ekf_yaw):.2f}° - "
            f"FAST-LIO: {math.degrees(self.last_fastlio_yaw):.2f}°)"
        )
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def transform_odometry(self, msg: Odometry, yaw_f: float) -> Odometry:
        """Transform FAST-LIO odometry to align with EKF frame"""
        aligned = Odometry()
        
        # Header
        aligned.header = msg.header
        aligned.header.frame_id = "odom"  # Change frame
        aligned.child_frame_id = "base_link"  # Change child frame
        
        # Extract position
        x_f = msg.pose.pose.position.x
        y_f = msg.pose.pose.position.y
        
        # Apply rotation to position (2D rotation matrix)
        cos_offset = math.cos(self.yaw_offset)
        sin_offset = math.sin(self.yaw_offset)
        
        x_aligned = x_f * cos_offset - y_f * sin_offset
        y_aligned = x_f * sin_offset + y_f * cos_offset
        
        # Apply offset to yaw
        yaw_aligned = yaw_f + self.yaw_offset
        
        # Set position (2D mode, z=0)
        aligned.pose.pose.position.x = x_aligned
        aligned.pose.pose.position.y = y_aligned
        aligned.pose.pose.position.z = 0.0
        
        # Set orientation (yaw only in 2D)
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw_aligned)
        aligned.pose.pose.orientation.x = quat[0]
        aligned.pose.pose.orientation.y = quat[1]
        aligned.pose.pose.orientation.z = quat[2]
        aligned.pose.pose.orientation.w = quat[3]
        
        # Covariance
        aligned.pose.covariance[0] = 0.01   # x
        aligned.pose.covariance[7] = 0.01   # y
        aligned.pose.covariance[14] = 0.01  # z
        aligned.pose.covariance[21] = 0.01  # roll
        aligned.pose.covariance[28] = 0.01  # pitch
        aligned.pose.covariance[35] = 0.01  # yaw
        
        return aligned


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOOdometryAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
