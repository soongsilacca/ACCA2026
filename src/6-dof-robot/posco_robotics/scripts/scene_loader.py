#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PlanningScene, CollisionObject
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import os

class SceneLoader(Node):
    def __init__(self):
        super().__init__('scene_loader')
        self.scene_pub = self.create_publisher(PlanningScene, 'planning_scene', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # Subscribe to Gazebo model states for sync
        from gazebo_msgs.msg import ModelStates
        self.model_state_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_state_callback,
            10
        )
        self.box_pose = None
        
        # Spawn client for Gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Wait for Gazebo to be ready
        self.create_timer(1.0, self.spawn_once)
        self.spawned = False
        
        # Publish ground plane and marker every 0.1 seconds (faster for sync)
        self.create_timer(0.1, self.publish_marker)
        self.create_timer(2.0, self.publish_scene) # Ground is static
        self.get_logger().info("Scene Loader initialized")

    def model_state_callback(self, msg):
        try:
            if 'red_box' in msg.name:
                idx = msg.name.index('red_box')
                self.box_pose = msg.pose[idx]
        except ValueError:
            pass

    def spawn_once(self):
        if self.spawned:
            return
            
        if not self.spawn_client.wait_for_service(timeout_sec=0.5):
            return
        
        # Delete existing red_box if it exists
        if self.delete_client.wait_for_service(timeout_sec=0.1):
            delete_req = DeleteEntity.Request()
            delete_req.name = 'red_box'
            self.delete_client.call_async(delete_req)
            self.get_logger().info('Deleted existing red_box')
            
        # Read SDF file
        from ament_index_python.packages import get_package_share_directory
        package_share = get_package_share_directory('posco_robotics')
        sdf_path = os.path.join(package_share, 'models', 'red_box', 'model.sdf')
        
        try:
            with open(sdf_path, 'r') as f:
                sdf_content = f.read()
        except Exception as e:
            self.get_logger().error(f'Failed to read SDF: {e}')
            return
        
        # Spawn request
        request = SpawnEntity.Request()
        request.name = 'red_box'
        request.xml = sdf_content
        request.robot_namespace = ''
        request.initial_pose = Pose()
        request.initial_pose.position.x = 0.5
        request.initial_pose.position.y = -0.2
        request.initial_pose.position.z = 0.025
        request.initial_pose.orientation.w = 1.0
        request.reference_frame = 'world'
        
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)
        self.spawned = True

    def spawn_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Red box spawned successfully in Gazebo')
            else:
                self.get_logger().error(f'Spawn failed: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Spawn service call failed: {e}')
    
    def publish_marker(self):
        if self.box_pose is None:
            # Fallback to initial if not yet found in Gazebo
            pose_x, pose_y, pose_z = 0.5, -0.2, 0.025
            orient_w = 1.0
            orient_x = 0.0
            orient_y = 0.0
            orient_z = 0.0
        else:
            pose_x = self.box_pose.position.x
            pose_y = self.box_pose.position.y
            pose_z = self.box_pose.position.z
            orient_w = self.box_pose.orientation.w
            orient_x = self.box_pose.orientation.x
            orient_y = self.box_pose.orientation.y
            orient_z = self.box_pose.orientation.z

        marker = Marker()
        marker.header.frame_id = "world" # Gazebo reports in world frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "red_box"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = pose_x
        marker.pose.position.y = pose_y
        marker.pose.position.z = pose_z
        marker.pose.orientation.x = orient_x
        marker.pose.orientation.y = orient_y
        marker.pose.orientation.z = orient_z
        marker.pose.orientation.w = orient_w
        
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.marker_pub.publish(marker)

    def publish_scene(self):
        # Ground Plane (Collision Object for MoveIt)
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        
        ground = CollisionObject()
        ground.header.frame_id = "link0" # Using link0 as base
        ground.id = "ground_plane"
        
        g_prim = SolidPrimitive()
        g_prim.type = SolidPrimitive.BOX
        g_prim.dimensions = [2.0, 2.0, 0.01]
        
        g_pose = Pose()
        g_pose.position.x = 0.0
        g_pose.position.y = 0.0
        # Thickness is 0.01 (1cm). Center at -0.005 puts top surface at 0.0
        g_pose.position.z = -0.005 
        g_pose.orientation.w = 1.0
        
        ground.primitives.append(g_prim)
        ground.primitive_poses.append(g_pose)
        ground.operation = CollisionObject.ADD
        
        scene_msg.world.collision_objects.append(ground)
        self.scene_pub.publish(scene_msg)

def main():
    rclpy.init()
    node = SceneLoader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()