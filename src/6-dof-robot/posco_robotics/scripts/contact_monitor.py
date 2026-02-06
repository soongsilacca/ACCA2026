#!/usr/bin/env python3
"""
Contact Monitor - Debug tool to see what contacts are being detected
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState

class ContactMonitor(Node):
    def __init__(self):
        super().__init__('contact_monitor')
        
        self.contact_sub = self.create_subscription(
            ContactsState,
            '/contact_sensor',
            self.contact_callback,
            10
        )
        
        self.get_logger().info('Contact Monitor started - watching /contact_sensor')
    
    def contact_callback(self, msg):
        """Print all contacts"""
        if len(msg.states) > 0:
            self.get_logger().info(f'=== CONTACTS DETECTED: {len(msg.states)} ===')
            for i, state in enumerate(msg.states):
                self.get_logger().info(f'Contact {i}:')
                self.get_logger().info(f'  collision1: {state.collision1_name}')
                self.get_logger().info(f'  collision2: {state.collision2_name}')
                self.get_logger().info(f'  depth: {state.depths[0] if state.depths else 0}')
        else:
            # Throttled message for no contacts
            self.get_logger().info('No contacts detected', throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = ContactMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
