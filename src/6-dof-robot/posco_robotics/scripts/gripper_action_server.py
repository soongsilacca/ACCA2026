#!/usr/bin/env python3
"""
Gripper Action Server for MoveIt Integration (Effort Controller)
Translates MoveIt gripper goals to effort commands
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import GripperCommand
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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
        
        # Publisher for trajectory commands (Position Control)
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('Gripper Action Server started (Position/Trajectory mode)')
        
    def execute_callback(self, goal_handle):
        position = goal_handle.request.command.position
        max_effort = goal_handle.request.command.max_effort
        
        self.get_logger().info(f'Executing goal: position={position:.4f}, max_effort={max_effort:.2f}')
        
        # Create Trajectory
        traj = JointTrajectory()
        traj.joint_names = ['finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        # point.effort = [max_effort] # Removed: Causes error in Gazebo controller
        point.time_from_start = Duration(sec=2, nanosec=0) # 2.0 second duration for smoother grasp
        
        traj.points = [point]
        self.traj_pub.publish(traj)
        
        # Wait for movement (simple sleep for now, better would be to monitor joint state)
        import time
        time.sleep(1.0)
        
        # Return success
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = position
        result.reached_goal = True
        result.stalled = False
        
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
