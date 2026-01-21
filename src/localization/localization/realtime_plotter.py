#!/usr/bin/env python3
"""
Real-time Localization Plotter
Visualizes GPS, Global Odometry, and Local Odometry in real-time.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

class RealtimePlotter(Node):
    def __init__(self):
        super().__init__('realtime_plotter')
        
        # Data Buffers (Max length to avoid memory issues?)
        # Let's keep it unlimited for now as run won't be infinite
        self.gps_x = []
        self.gps_y = []
        
        self.global_x = []
        self.global_y = []
        
        self.local_x = []
        self.local_y = []
        
        # Alignment
        self.gps_start = None
        self.global_start = None
        self.local_start = None
        
        # Subscriptions
        self.create_subscription(Odometry, '/gps/odometry_nwu', self.gps_callback, 10)
        self.create_subscription(Odometry, '/odometry/global', self.global_callback, 10)
        self.create_subscription(Odometry, '/odometry/local', self.local_callback, 10)
        
        self.get_logger().info("Real-time Plotter Started.")

    def gps_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.gps_x.append(x)
        self.gps_y.append(y)
        
        if self.gps_start is None:
            self.gps_start = (x, y)

    def global_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.global_x.append(x)
        self.global_y.append(y)

    def local_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Align Local to GPS Start
        if self.local_start is None:
            if self.gps_start is None:
                return # Wait for GPS to align
            self.local_start = (x, y)
            
        if self.local_start and self.gps_start:
            # Shift Local to start where GPS started
            aligned_x = (x - self.local_start[0]) + self.gps_start[0]
            aligned_y = (y - self.local_start[1]) + self.gps_start[1]
            
            self.local_x.append(aligned_x)
            self.local_y.append(aligned_y)

def spin_thread(node):
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Spin failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RealtimePlotter()
    
    # Run ROS spin in separate thread
    t = threading.Thread(target=spin_thread, args=(node,))
    t.start()
    
    # Set up Plotting
    fig, ax = plt.subplots()
    ax.set_title("Real-time Localization Trajectory")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True)
    ax.set_aspect('equal')
    
    # Lines
    line_gps, = ax.plot([], [], 'r-', label='GPS (Ground Truth)', alpha=0.6)
    line_global, = ax.plot([], [], 'b-', label='Global EKF')
    line_local, = ax.plot([], [], 'g--', label='Local Odom (Aligned)')
    ax.legend(loc='upper right')
    
    def update(frame):
        # Update Data
        line_gps.set_data(node.gps_x, node.gps_y)
        line_global.set_data(node.global_x, node.global_y)
        line_local.set_data(node.local_x, node.local_y)
        
        # Rescale axes
        all_x = node.gps_x + node.global_x + node.local_x
        all_y = node.gps_y + node.global_y + node.local_y
        
        if all_x and all_y:
            min_x, max_x = min(all_x), max(all_x)
            min_y, max_y = min(all_y), max(all_y)
            
            # Add padding
            margin = 2.0
            ax.set_xlim(min_x - margin, max_x + margin)
            ax.set_ylim(min_y - margin, max_y + margin)
            
        return line_gps, line_global, line_local
    
    ani = FuncAnimation(fig, update, interval=100) # 10Hz update
    
    try:
        plt.show() # Blocks here until window closed
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()
    t.join(timeout=1.0)

if __name__ == '__main__':
    main()
