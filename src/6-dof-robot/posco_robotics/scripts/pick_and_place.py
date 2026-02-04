#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        # Enable usage of simulation time to sync with Gazebo/TF
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.planning_scene_publisher = self.create_publisher(PlanningScene, 'planning_scene', 10)
        self.apply_planning_scene_client = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        
        self.get_logger().info("Waiting for MoveGroup action server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("MoveGroup action server available.")

    def add_collision_object(self):
        # Create a red box collision object
        co = CollisionObject()
        co.header.frame_id = "link0" # Using link0 which is the robot base
        co.id = "red_box"
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.05]
        
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.0
        pose.position.z = 0.025
        pose.orientation.w = 1.0
        
        co.primitives.append(primitive)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(co)
        planning_scene.is_diff = True
        
        self.planning_scene_publisher.publish(planning_scene)
        self.get_logger().info("Published red_box to planning_scene")

    def add_ground_plane(self):
        # Add ground plane to prevent controller from smashing into the floor
        co = CollisionObject()
        co.header.frame_id = "link0"
        co.id = "ground_plane"
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [2.0, 2.0, 0.01] # 2x2m floor
        
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = -0.006 # Slightly below 0 to allow wheels/base to sit? 
        # Indy7 base is at z=0. Link0 visual mesh might be fine, but let's put floor at z=-0.01 roughly.
        # Actually, Gazebo ground is at 0. Link0 origin is at 0.
        # If we put collision object at 0, it might collide with base.
        # Let's put it at z=-0.05
        pose.position.z = -0.05
        pose.orientation.w = 1.0
        
        co.primitives.append(primitive)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(co)
        planning_scene.is_diff = True
        
        self.planning_scene_publisher.publish(planning_scene)
        self.get_logger().info("Published ground_plane to planning_scene")

    def construct_pose_goal(self, group_name, target_pose, tolerance_pos=0.01, tolerance_ori=0.01):
        goal_msg = MoveGroup.Goal()
        
        request = goal_msg.request
        request.group_name = group_name
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.2 # SLOW DOWN for safety
        request.max_acceleration_scaling_factor = 0.2 # SLOW DOWN for safety
        
        # Define Constraints
        c = Constraints()
        c.name = "pose_goal"
        
        # Position constraint
        pc = PositionConstraint()
        pc.header.frame_id = "link0" # Ensure using link0
        pc.link_name = "tcp"
        
        bv = BoundingVolume()
        pr = SolidPrimitive()
        pr.type = SolidPrimitive.BOX
        pr.dimensions = [tolerance_pos, tolerance_pos, tolerance_pos]
        bv.primitives.append(pr)
        
        bv_pose = Pose()
        bv_pose.position = target_pose.position
        bv_pose.orientation.w = 1.0
        bv.primitive_poses.append(bv_pose)
        
        pc.constraint_region = bv
        pc.weight = 1.0
        
        # Orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = "link0"
        oc.link_name = "tcp"
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = tolerance_ori
        oc.absolute_y_axis_tolerance = tolerance_ori
        oc.absolute_z_axis_tolerance = tolerance_ori
        oc.weight = 1.0
        
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        
        request.goal_constraints.append(c)
        
        return goal_msg

    def construct_joint_goal(self, group_name, joint_names, joint_values):
        goal_msg = MoveGroup.Goal()
        request = goal_msg.request
        request.group_name = group_name
        request.max_velocity_scaling_factor = 0.2 # SLOW DOWN
        request.max_acceleration_scaling_factor = 0.2 # SLOW DOWN
        
        c = Constraints()
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
            
        request.goal_constraints.append(c)
        return goal_msg

    def send_goal_and_wait(self, goal_msg):
        self.get_logger().info(f"Sending goal for {goal_msg.request.group_name}...")
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
            
        self.get_logger().info('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        if result.error_code.val == 1: # SUCCESS
            self.get_logger().info('Goal succeeded!')
            return True
        else:
            self.get_logger().error(f'Goal failed with error code: {result.error_code.val}')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    
    # 1. Add Environment
    node.add_ground_plane() # ADD GROUND
    node.add_collision_object()
    import time
    time.sleep(2.0) # Wait for scene update

    # 2. Move to Ready Pose (High Safety Pose)
    node.get_logger().info("Moving to Ready Pose...")
    # Higher up and retracted to avoid hitting the box or table
    ready_joints = [0.0, -0.5, -1.57, 0.0, -1.0, 0.0] 
    goal = node.construct_joint_goal("arm", [f"joint{i}" for i in range(6)], ready_joints)
    node.send_goal_and_wait(goal)
    
    # 3. Open Gripper
    node.get_logger().info("Opening Gripper...")
    goal = node.construct_joint_goal("gripper", ["finger_joint"], [0.0])
    node.send_goal_and_wait(goal)
    
    # --- GET CURRENT POSE VIA TF ---
    from tf2_ros import Buffer, TransformListener
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    
    # --- GET CURRENT POSE VIA TF ---
    from tf2_ros import Buffer, TransformListener
    
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    # Spin for a bit to populate TF buffer
    # We must spin the node to receive TF messages
    node.get_logger().info("Waiting for TF data...")
    end_time = node.get_clock().now().nanoseconds / 1e9 + 2.0
    while node.get_clock().now().nanoseconds / 1e9 < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    current_orientation = None
    try:
        # Check "tcp" relative to "link0"
        # msg: transform from source_frame (tcp) to target_frame (link0)
        trans = tf_buffer.lookup_transform('link0', 'tcp', rclpy.time.Time())
        current_orientation = trans.transform.rotation
        node.get_logger().info(f"Captured Current Orientation: {current_orientation}")
    except Exception as e:
        node.get_logger().error(f"Could not get transform: {e}")
            
    if current_orientation is None:
        node.get_logger().error("Failed to get transform. Using fallback.")
        current_orientation = Quaternion()
        current_orientation.x = 0.0
        current_orientation.y = 1.0
        current_orientation.z = 0.0
        current_orientation.w = 0.0 # 180 deg rotation about Y

    # 4. Pre-Grasp Pose
    pre_grasp = Pose()
    pre_grasp.position.x = 0.5
    pre_grasp.position.y = 0.0
    pre_grasp.position.z = 0.45 # Approach height
    pre_grasp.orientation = current_orientation # USE CAPTURED ORIENTATION
    
    node.get_logger().info("Moving to Pre-Grasp...")
    goal = node.construct_pose_goal("arm", pre_grasp, tolerance_pos=0.05, tolerance_ori=0.1)
    node.send_goal_and_wait(goal)
    
    # 5. Grasp Pose
    grasp = Pose()
    grasp.position.x = 0.5
    grasp.position.y = 0.0
    grasp.position.z = 0.23 # Grasp height
    grasp.orientation = pre_grasp.orientation
    
    node.get_logger().info("Moving to Grasp...")
    goal = node.construct_pose_goal("arm", grasp, tolerance_pos=0.02, tolerance_ori=0.1)
    node.send_goal_and_wait(goal)
    
    # 6. Close Gripper
    node.get_logger().info("Closing Gripper...")
    goal = node.construct_joint_goal("gripper", ["finger_joint"], [0.725])
    node.send_goal_and_wait(goal)
    
    # 7. Lift
    lift = Pose()
    lift.position.x = 0.5
    lift.position.y = 0.0
    lift.position.z = 0.45
    lift.orientation = pre_grasp.orientation
    
    node.get_logger().info("Lifting...")
    goal = node.construct_pose_goal("arm", lift, tolerance_pos=0.05, tolerance_ori=0.1)
    node.send_goal_and_wait(goal)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
