#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class MimicJointPlugin(Node):
    def __init__(self):
        super().__init__('mimic_joint_plugin')
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Publisher for mimic controller
        self.mimic_pub = self.create_publisher(
            Float64MultiArray,
            '/mimic_joint_controller/commands',
            10
        )
        
        self.mimic_joints = [
            'left_inner_knuckle_joint',
            'left_inner_finger_joint',
            'right_outer_knuckle_joint',
            'right_inner_knuckle_joint',
            'right_inner_finger_joint'
        ]
        
        # Multipliers based on reference package
        self.multipliers = {
            'left_inner_knuckle_joint': 1.0,
            'left_inner_finger_joint': -1.0,
            'right_outer_knuckle_joint': -1.0,
            'right_inner_knuckle_joint': -1.0,
            'right_inner_finger_joint': 1.0
        }
        
        self.get_logger().info('Mimic Joint Plugin Initialized')

    def joint_callback(self, msg):
        try:
            if 'finger_joint' in msg.name:
                idx = msg.name.index('finger_joint')
                finger_pos = msg.position[idx]
                
                command_msg = Float64MultiArray()
                # Order must match ros2_controllers.yaml "joints" list
                # YAML order:
                # - left_inner_knuckle_joint
                # - left_inner_finger_joint
                # - right_outer_knuckle_joint
                # - right_inner_knuckle_joint
                # - right_inner_finger_joint
                
                cmds = []
                cmds.append(finger_pos * self.multipliers['left_inner_knuckle_joint'])
                cmds.append(finger_pos * self.multipliers['left_inner_finger_joint'])
                cmds.append(finger_pos * self.multipliers['right_outer_knuckle_joint'])
                cmds.append(finger_pos * self.multipliers['right_inner_knuckle_joint'])
                cmds.append(finger_pos * self.multipliers['right_inner_finger_joint'])
                
                command_msg.data = cmds
                self.mimic_pub.publish(command_msg)
                
        except ValueError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MimicJointPlugin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
