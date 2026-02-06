#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription_ = self.create_subscription(Odometry, '/odometry/local_map_aligned', self.odom_callback, 10)
        self.sent = False
        self.get_logger().info('Waiting for Odom to set goal...')

    def odom_callback(self, msg):
        if self.sent:
            return
            
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        
        # Set goal = current position to trigger loop planning
        goal_msg.pose = msg.pose.pose
        
        self.get_logger().info(f'Received Odom ({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f}). Publishing as GOAL for Loop.')
        self.publisher_.publish(goal_msg)
        self.sent = True

def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    
    # Spin until sent, then wait a bit and exit
    start = node.get_clock().now()
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.sent:
             # Wait 1 more second to ensure message goes out
             duration = node.get_clock().now() - start
             if duration.nanoseconds / 1e9 > 5.0: # Timeout or success
                 break
        else:
             # Timeout if no odom received
             duration = node.get_clock().now() - start
             if duration.nanoseconds / 1e9 > 10.0:
                 node.get_logger().error("Timeout waiting for Odom!")
                 break
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
