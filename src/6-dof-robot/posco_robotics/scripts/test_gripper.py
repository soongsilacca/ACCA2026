#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class GripperActionTest(Node):
    def __init__(self):
        super().__init__('gripper_action_test')
        
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready!')
        
    def send_goal(self, position):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f'Sending goal: finger_joint = {position}')
        future = self._action_client.send_goal_async(goal_msg)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = GripperActionTest()
    
    print("\nGripper Action Test - Commands:")
    print("  o - Open gripper (0.0)")
    print("  c - Close gripper (0.725)")
    print("  q - Quit\n")
    
    try:
        while rclpy.ok():
            cmd = input("Enter command: ").strip().lower()
            
            if cmd == 'o':
                future = node.send_goal(0.0)
                rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            elif cmd == 'c':
                future = node.send_goal(0.725)
                rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            elif cmd == 'q':
                break
            else:
                print("Invalid command. Use 'o', 'c', or 'q'")
                
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
