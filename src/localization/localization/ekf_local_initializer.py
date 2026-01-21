#!/usr/bin/env python3
"""
EKF Local Initializer

Subscribes to /navheading (IMU) to get initial robot orientation.
Sets initial pose of EKF Local via /set_pose/local.
This ensures odom frame is aligned with map frame (North-aligned) instantly.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import sys

class EKFLocalInitializer(Node):
    def __init__(self):
        super().__init__('ekf_local_initializer')
        
        self.initialized = False
        
        # Subscribe to /navheading for initial orientation
        # self.sub_imu = self.create_subscription(
        #     Imu, '/navheading', self.imu_callback, 10
        # )
        
        # Publisher for EKF Local set_pose
        self.pub_set_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/set_pose/local', 10
        )
        
        self.get_logger().info('EKF Local Initializer Started. Waiting for /navheading...')
        
    def imu_callback(self, msg: Imu):
        if self.initialized:
            return
            
        # Create initial pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'
        
        # Position is 0,0,0 (start point is odom origin)
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        
        # Orientation from IMU (Robot's actual global heading)
        # CONVERSION: User Map X = North (NWU frame)
        # IMU is likely ENU (East=0, North=90) or Compass (North=0)
        # Assuming ENU per typical ROS IMU:
        # We need to subtract 90 degrees so that North (90) becomes 0 (X-axis)
        
        # 1. Convert IMU quaternion to Yaw
        q_imu = msg.orientation
        siny_cosp = 2.0 * (q_imu.w * q_imu.z + q_imu.x * q_imu.y)
        cosy_cosp = 1.0 - 2.0 * (q_imu.y * q_imu.y + q_imu.z * q_imu.z)
        yaw_imu = math.atan2(siny_cosp, cosy_cosp)
        
        # 2. Subtract 90 degrees (pi/2) for NWU alignment
        yaw_nwu = yaw_imu - (math.pi / 2.0)
        
        # Normalize to -pi to pi
        while yaw_nwu > math.pi:
            yaw_nwu -= 2.0 * math.pi
        while yaw_nwu < -math.pi:
            yaw_nwu += 2.0 * math.pi
            
        # 3. Convert back to Quaternion
        cy = math.cos(yaw_nwu * 0.5)
        sy = math.sin(yaw_nwu * 0.5)
        
        pose_msg.pose.pose.orientation.w = cy
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = sy
        
        # Covariance
        for i in range(36):
            pose_msg.pose.covariance[i] = 0.0
            
        # Set diagonal values
        pose_msg.pose.covariance[0] = 1e-9  # x
        pose_msg.pose.covariance[7] = 1e-9  # y
        pose_msg.pose.covariance[14] = 1e-9 # z
        pose_msg.pose.covariance[21] = 0.01 # roll
        pose_msg.pose.covariance[28] = 0.01 # pitch
        pose_msg.pose.covariance[35] = 0.01 # yaw (trust IMU)
        
        self.pub_set_pose.publish(pose_msg)
        
        self.get_logger().info(f'Initialized EKF Local with yaw_nwu: {math.degrees(yaw_nwu):.2f}° (IMU was {math.degrees(yaw_imu):.2f}°)')
        
        self.initialized = True
        self.get_logger().info('Initialization complete. Node shutting down.')
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    except Exception as e:
         pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
