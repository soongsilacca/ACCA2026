#!/usr/bin/env python3
"""
Gripper Action Server for MoveIt Integration
Translates MoveIt gripper goals to gripper_controller commands
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import GripperCommand
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class GripperActionServer(Node):
    def __init__(self):
        super().__init__('gripper_action_server')
        
        # Action server for MoveIt
        self._action_server = ActionServer(
            self,
            GripperCommand,
            'gripper_controller/gripper_cmd',
            self.execute_callback
        )
        
        # Action client to real controller
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Gripper Action Server started')
        self._gripper_client.wait_for_server()
        
    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: position={goal_handle.request.command.position}')
        
        # Translate GripperCommand to JointTrajectory
        traj_goal = FollowJointTrajectory.Goal()
        traj_goal.trajectory.joint_names = ['finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [goal_handle.request.command.position]
        point.time_from_start = Duration(sec=2, nanosec=0)
        traj_goal.trajectory.points = [point]
        
        # Send to gripper controller
        future = self._gripper_client.send_goal_async(traj_goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_result = future.result()
        if not goal_result.accepted:
            goal_handle.abort()
            return GripperCommand.Result()
            
        # Wait for completion
        result_future = goal_result.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        # Return success
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = goal_handle.request.command.position
        result.reached_goal = True
        
        self.get_logger().info('Goal completed')
        return result

def main(args=None):
    rclpy.init(args=args)
    server = GripperActionServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
