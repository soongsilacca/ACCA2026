#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry, Path
from erp42_msgs.msg import ControlMessage
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
import numpy as np
import math as m
from stanley import Stanley

def euler_from_quaternion(quaternion):
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class State:
    def __init__(self, x=0, y=0, yaw=0, v=0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.odom_sub = self.create_subscription(Odometry, '/odometry/global', self.odom_callback, qos_profile_system_default)
        self.path_sub = self.create_subscription(Path, '/global_path', self.path_callback, qos_profile_system_default)
        self.enable_sub = self.create_subscription(Bool, '/pure_pursuit/enable', self.enable_callback, qos_profile_system_default)
        self.cmd_pub = self.create_publisher(ControlMessage, '/cmd_msg', qos_profile_system_default)
        self.target_pub = self.create_publisher(Marker, '/stanley_target', qos_profile_system_default)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.current_state = State()
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.stanley = Stanley()
        self.has_path = False
        self.enabled = True # Enabled by default for testing
        self.alive_cnt = 0

    def odom_callback(self, msg):
        # self.get_logger().info('Received Odom') # Debug log (too verbose, commenting out)
        self.current_state.x = msg.pose.pose.position.x
        self.current_state.y = msg.pose.pose.position.y
        _, _, self.current_state.yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        # Assuming linear velocity available
        self.current_state.v = m.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)

    def path_callback(self, msg):
        if not msg.poses:
            return
        self.cx = []
        self.cy = []
        self.cyaw = []
        for pose in msg.poses:
            self.cx.append(pose.pose.position.x)
            self.cy.append(pose.pose.position.y)
            _, _, yaw = euler_from_quaternion([
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            ])
            self.cyaw.append(yaw)
        self.has_path = True
        self.get_logger().info('Received new path with {} points'.format(len(self.cx)))
        
    def enable_callback(self, msg):
        if msg.data and not self.enabled:
             self.get_logger().info("Path Tracking ENABLED")
        elif not msg.data and self.enabled:
             self.get_logger().info("Path Tracking DISABLED")
        self.enabled = msg.data

    def control_loop(self):
        if not self.enabled or not self.has_path or len(self.cx) < 2:
            return

        delta, target_idx, hdr, ctr = self.stanley.stanley_control(self.current_state, self.cx, self.cy, self.cyaw, h_gain=0.5, c_gain=0.24)
        
        msg = ControlMessage()
        msg.mora = 1 # Auto mode
        msg.estop = 0 # No E-stop
        msg.gear = 2 # Forward (0:B, 1:N, 2:D)
        msg.speed = 100 # 15.0 KPH (Input is KPH * 10) - Increased for faster driving
        msg.steer = int(m.degrees((-1) * delta) * 1000) # Millidegrees
        msg.brake = 0
        msg.alive = self.alive_cnt
        self.alive_cnt = (self.alive_cnt + 1) % 256
        
        # Safety check? 
        
        # Visualize Target Point
        if target_idx < len(self.cx):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "stanley_target"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.cx[target_idx]
            marker.pose.position.y = self.cy[target_idx]
            marker.pose.position.z = 1.5 # Lift it higher for visibility
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.5 # Larger
            marker.scale.y = 2.5
            marker.scale.z = 2.5
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0 # Bright Green
            marker.color.b = 0.0
            # self.target_pub.publish(marker)

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
