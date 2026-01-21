#!/usr/bin/env python3
"""
Odom Map Republisher

Subscribes to /odometry/local (odom frame)
Republishes to /odometry/local_map_aligned with frame_id='map'

This allows EKF Global to fuse the Local EKF's orientation as if it were absolute Map orientation,
forcing T_map_base orientation == T_odom_base orientation,
thus ensuring T_map_odom has no rotation.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomMapRepublisher(Node):
    def __init__(self):
        super().__init__('odom_map_republisher')
        
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/local', self.odom_callback, 10
        )
        
        self.pub_odom = self.create_publisher(
            Odometry, '/odometry/local_map_aligned', 10
        )
        
        self.get_logger().info('Odom Map Republisher Started. Redirecting frame odom -> map.')

    def odom_callback(self, msg: Odometry):
        # Create new message
        new_msg = Odometry()
        new_msg.header = msg.header
        new_msg.header.frame_id = 'map'  # SPOOF: Claim this is in Map frame
        new_msg.child_frame_id = msg.child_frame_id
        
        # Copy data
        new_msg.pose = msg.pose
        new_msg.twist = msg.twist
        
        # We only really care about Orientation for this trick, but passing all helps debug
        
        self.pub_odom.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomMapRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
