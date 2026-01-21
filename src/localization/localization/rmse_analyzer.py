#!/usr/bin/env python3
"""
RMSE Analyzer Node
Calculates RMSE for Global and Local Odometry against GPS Ground Truth.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import numpy as np

class RMSEAnalyzer(Node):
    def __init__(self):
        super().__init__('rmse_analyzer')
        
        # Data Storage
        self.gps_pose = None
        self.global_pose = None
        self.local_pose = None
        
        self.gps_history = []  # Stores (timestamp, x, y)
        
        # Error Accumulators
        self.global_sq_error_sum = 0.0
        self.global_count = 0
        
        self.local_sq_error_sum = 0.0
        self.local_count = 0
        
        # Alignment offsets (Local needs to be aligned to start point)
        self.gps_start = None
        self.global_start = None
        self.local_start = None
        
        # Subscriptions
        self.create_subscription(Odometry, '/gps/odometry_nwu', self.gps_callback, 10)
        self.create_subscription(Odometry, '/odometry/global', self.global_callback, 10)
        self.create_subscription(Odometry, '/odometry/local', self.local_callback, 10)
        
        # Timer for output
        self.create_timer(1.0, self.print_metrics)
        
        self.get_logger().info("RMSE Analyzer Started. Waiting for data...")

    def gps_callback(self, msg):
        self.gps_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.gps_start is None:
            self.gps_start = self.gps_pose
            self.get_logger().info(f"GPS Start Set: {self.gps_start}")
            
    def global_callback(self, msg):
        # Global is in map frame, same as GPS. Compare directly.
        if self.gps_pose is None:
            return
            
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.global_pose = (x, y)
        
        if self.global_start is None:
            self.global_start = (x, y)
            self.get_logger().info(f"Global Start Set: {self.global_start}")
        
        # Simple synchronization: compare to MOST RECENT GPS pose
        # (Assuming 50Hz Odom vs 10Hz GPS, error is negligible for this purpose)
        dx = x - self.gps_pose[0]
        dy = y - self.gps_pose[1]
        
        sq_error = dx*dx + dy*dy
        self.global_sq_error_sum += sq_error
        self.global_count += 1
        
    def local_callback(self, msg):
        # Local starts at (0,0). Must align to GPS start.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.local_start is None:
            if self.gps_start is None:
                return # Can't align yet
            self.local_start = (x, y)
            self.get_logger().info(f"Local Start Set: {self.local_start} (Aligned to GPS Start)")
            
        if self.gps_pose is None:
            return

        # Transform Local to 'Aligned Map Frame'
        # Simple Translation alignment (ignoring rotation diff for now, assuming Odom/Map aligned)
        # aligned_x = (local_x - local_start_x) + gps_start_x
        aligned_x = (x - self.local_start[0]) + self.gps_start[0]
        aligned_y = (y - self.local_start[1]) + self.gps_start[1]
        
        dx = aligned_x - self.gps_pose[0]
        dy = aligned_y - self.gps_pose[1]
        
        sq_error = dx*dx + dy*dy
        self.local_sq_error_sum += sq_error
        self.local_count += 1

    def print_metrics(self):
        if self.global_count == 0:
            return
            
        global_rmse = math.sqrt(self.global_sq_error_sum / self.global_count)
        
        local_rmse = 0.0
        if self.local_count > 0:
            local_rmse = math.sqrt(self.local_sq_error_sum / self.local_count)
            
        print(f"--- Localization RMSE Analysis ---")
        print(f"Global RMSE: {global_rmse:.4f} m (samples: {self.global_count})")
        print(f"Local  RMSE: {local_rmse:.4f} m (samples: {self.local_count})")
        
        # Calculate Loop Closure Error (Distance from Start)
        if self.global_pose and self.global_start:
            # Current distance from Global Start
            dx = self.global_pose[0] - self.global_start[0]
            dy = self.global_pose[1] - self.global_start[1]
            global_dist = math.sqrt(dx*dx + dy*dy)
            print(f"Global Dist from Start: {global_dist:.4f} m")
            
        if self.local_pose and self.local_start:
            # Current distance from Local Start (Raw local frame)
            # Local start is usually (0,0) but let's be safe
            dx = self.local_pose[0] - self.local_start[0]
            dy = self.local_pose[1] - self.local_start[1]
            local_dist = math.sqrt(dx*dx + dy*dy)
            print(f"Local  Dist from Start: {local_dist:.4f} m")

        print(f"----------------------------------")

def main(args=None):
    rclpy.init(args=args)
    node = RMSEAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
