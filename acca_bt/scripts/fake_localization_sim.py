#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from erp42_msgs.msg import ControlMessage
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class FakeLocalizationSim(Node):
    def __init__(self):
        super().__init__('fake_localization_sim')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odometry/global', qos_profile_system_default)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.initialized = False
        self.create_subscription(ControlMessage, '/cmd_msg', self.cmd_callback, qos_profile_system_default)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.init_pose_callback, qos_profile_system_default)
        
        from nav_msgs.msg import Path
        from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Path, '/global_path', self.path_callback, latching_qos)

        # Parameters
        self.dt = 0.05  # 20Hz update
        self.wheelbase = 1.24
        
        # State [x, y, yaw, v]
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        
        # Inputs
        self.target_v = 0.0
        self.steering_angle = 0.0 # radians

        self.timer = self.create_timer(self.dt, self.update_state)
        self.get_logger().info("Fake Localization Sim Started. Waiting for /global_path to auto-initialize...")

    def path_callback(self, msg):
        if not msg.poses: return
        
        # Only initialize once.
        # Removing 'dist > 10.0' check because it causes reset when driving on a static path.
        if not self.initialized:
            start_pose = msg.poses[0].pose
            self.x = start_pose.position.x
            self.y = start_pose.position.y
            
            # Simple assumption: point towards the second point
            if len(msg.poses) > 1:
                p2 = msg.poses[1].pose
                self.yaw = math.atan2(p2.position.y - self.y, p2.position.x - self.x)
            
            self.v = 0.0 # Reset velocity
            self.target_v = 0.0
            
            self.initialized = True
            self.get_logger().info(f"Auto-initialized pose to start of Global Path: {self.x:.2f}, {self.y:.2f}")

    def cmd_callback(self, msg):
        # Decode ControlMessage
        # speed: KPH * 10 -> m/s
        kph = msg.speed / 10.0
        self.target_v = kph / 3.6
        
        # steer: millidegrees -> radians
        # msg.steer is inverted in path_follower: int(m.degrees((-1) * delta) * 1000)
        # So we invert back
        deg = - (msg.steer / 1000.0) 
        self.steering_angle = math.radians(deg)
        
        # Simple brake logic
        if msg.brake > 0 or msg.estop == 1:
            self.target_v = 0.0

    def init_pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.v = 0.0
        self.get_logger().info(f"Teleported to: {self.x:.2f}, {self.y:.2f}")

    def update_state(self):
        # Simple Bicycle Model
        # acceleration
        if self.v < self.target_v:
            self.v += 2.0 * self.dt # 2 m/s^2 accel
        elif self.v > self.target_v:
            self.v -= 4.0 * self.dt # 4 m/s^2 decel
            
        if self.v < 0: self.v = 0
            
        # Kinematics
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += (self.v / self.wheelbase) * math.tan(self.steering_angle) * self.dt
        
        # Normalize Yaw
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Publish Odom
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Yaw to Quaternion
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.z = sy
        
        odom.twist.twist.linear.x = self.v
        
        self.odom_pub.publish(odom)

        # Publish TF for Rviz Visualization
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = cy
        t.transform.rotation.z = sy
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FakeLocalizationSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
