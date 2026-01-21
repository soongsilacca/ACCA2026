#!/usr/bin/env python3
"""
IMU NWU Adapter

Rotates IMU data by -90 degrees (Yaw) to convert from ENU (North=90) to NWU (North=0).
This ensures EKF works with X=North convention.
Subscribes: /imu/data
Publishes: /imu/data_nwu
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from geometry_msgs.msg import Quaternion

class ImuNwuAdapter(Node):
    def __init__(self):
        super().__init__('imu_nwu_adapter')
        
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        
        self.pub_imu = self.create_publisher(
            Imu, '/imu/data_nwu', 10
        )
        
        # Quaternion for -90 degree rotation around Z axis
        # Euler: roll=0, pitch=0, yaw=-pi/2
        # qx = 0, qy = 0, qz = sin(-pi/4), qw = cos(-pi/4)
        # sin(-45) = -0.7071, cos(-45) = 0.7071
        self.q_rot_z = -0.70710678
        self.q_rot_w = 0.70710678
        
        self.get_logger().info('IMU NWU Adapter Started. Converting ENU(North=90) to NWU(North=0).')

    def imu_callback(self, msg: Imu):
        new_msg = Imu()
        new_msg.header = msg.header
        # Check if frame_id needs changing? EKF usually ignores frame_id if we rely on config
        # But let's keep it or append/modify if needed.
        
        # Copy covariance, but we must rotate it too!
        # Initial copy
        cov_ori = list(msg.orientation_covariance)
        cov_vel = list(msg.angular_velocity_covariance)
        cov_acc = list(msg.linear_acceleration_covariance)

        # Rotate diagonal elements (Variance) for X and Y
        # Index 0 (XX) <-> Index 4 (YY)
        
        # Orientation
        cov_ori[0], cov_ori[4] = cov_ori[4], cov_ori[0]
        
        # Angular Velocity
        cov_vel[0], cov_vel[4] = cov_vel[4], cov_vel[0]
        
        # Linear Acceleration
        cov_acc[0], cov_acc[4] = cov_acc[4], cov_acc[0]
        
        new_msg.orientation_covariance = cov_ori
        new_msg.angular_velocity_covariance = cov_vel
        new_msg.linear_acceleration_covariance = cov_acc
        
        # 1. Rotate Orientation
        # q_new = q_orig * q_rot
        # Note: We want to rotate the FRAME? or the READING?
        # If robot is at 90 (North), we want it to read 0.
        # So we apply a rotation that maps 90 -> 0.
        # That is -90 degrees.
        # q_new = q_meas * q_correction
        
        q = msg.orientation
        
        # Quaternion multiplication: q_new = q * rot
        # (w1, x1, y1, z1) * (w2, x2, y2, z2)
        # w = w1w2 - x1x2 - y1y2 - z1z2
        # x = w1x2 + x1w2 + y1z2 - z1y2
        # y = w1y2 - x1z2 + y1w2 + z1x2
        # z = w1z2 + x1y2 - y1x2 + z1w2
        
        # My rot: (w=0.707, x=0, y=0, z=-0.707)
        w2 = self.q_rot_w
        x2 = 0.0
        y2 = 0.0
        z2 = self.q_rot_z
        
        new_msg.orientation.w = q.w*w2 - q.z*z2
        new_msg.orientation.x = q.x*w2 + q.y*z2
        new_msg.orientation.y = q.y*w2 - q.x*z2
        new_msg.orientation.z = q.z*w2 + q.w*z2
        
        # 2. Angular Velocity
        # Z-axis angular velocity remains the same (rotation around Z doesn't change spin rate around Z)
        # However, X and Y components rotate.
        # Since we effectively redefined X->Y, Y->-X
        # v_x_new = v_y_old
        # v_y_new = -v_x_old
        # Verify: Old=North(Y). We want New=North(X).
        # Linear velocity along North is +Y in Old. +X in New.
        # So X_new = Y_old.
        # Y_new (West) = -X_old (West is -X in ENU? No, X is East. So -East = West. Correct).
        # So:
        new_msg.angular_velocity.x = msg.angular_velocity.y
        new_msg.angular_velocity.y = -msg.angular_velocity.x
        new_msg.angular_velocity.z = msg.angular_velocity.z
        
        # 3. Linear Acceleration
        # Same rotation logic
        new_msg.linear_acceleration.x = msg.linear_acceleration.y
        new_msg.linear_acceleration.y = -msg.linear_acceleration.x
        new_msg.linear_acceleration.z = msg.linear_acceleration.z
        
        self.pub_imu.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNwuAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
