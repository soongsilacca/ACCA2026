#!/usr/bin/env python3
"""
Gripper Grasp Controller
Stops gripper when BOTH fingers contact object
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState, EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tf2_ros
from geometry_msgs.msg import TransformStamped

class GripperGraspController(Node):
    def __init__(self):
        super().__init__('gripper_grasp_controller')
        
        self.contact_sub = self.create_subscription(
            ContactsState,
            '/indy7/contact_sensor',
            self.contact_callback,
            10
        )
        
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )
        
        # TF2 for gripper pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Gazebo services for object attachment
        self.get_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # Wait for service to be available
        self.get_logger().info('Waiting for /gazebo/set_entity_state service...')
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info('✅ SetEntityState service ready!')
        
        self.current_gripper_position = 0.0
        self.left_contact = False
        self.right_contact = False
        self.gripper_stopped = False
        self.contact_stable_count = 0
        self.object_attached = False
        
        # Timer for updating attached object (100Hz)
        self.create_timer(0.01, self.update_attached_object)
        
        self.get_logger().info('Gripper Grasp Controller - waiting for BOTH finger contacts')
    
    def joint_callback(self, msg):
        try:
            idx = msg.name.index('finger_joint')
            current_pos = msg.position[idx]
            
            # Reset grasp state when gripper opens (position < 0.1)
            # 0.0 is open, 0.8 is closed
            if current_pos < 0.1 and self.gripper_stopped:
                self.gripper_stopped = False
                self.contact_stable_count = 0
                self.object_attached = False
                self.get_logger().info('🔄 Gripper OPENED - Resetting grasp state for next pick')
                
            self.current_gripper_position = current_pos
        except (ValueError, IndexError):
            pass
    
    def contact_callback(self, msg):
        """Detect contact on both fingers and stop gripper"""
        import time
        current_time = time.time()
        
        # Update contact state based on this message
        if len(msg.states) > 0:
            for state in msg.states:
                if 'red_box' in state.collision1_name or 'red_box' in state.collision2_name:
                    contact_info = state.collision1_name + state.collision2_name
                    
                    if 'left_inner_finger' in contact_info:
                        self.left_contact = True
                        self.left_contact_time = current_time
                    if 'right_inner_finger' in contact_info:
                        self.right_contact = True
                        self.right_contact_time = current_time
        
        # Check if both contacts are recent (within 0.2 seconds)
        if hasattr(self, 'left_contact_time') and hasattr(self, 'right_contact_time'):
            if (current_time - self.left_contact_time < 0.2 and 
                current_time - self.right_contact_time < 0.2):
                
                self.contact_stable_count += 1
                
                if self.contact_stable_count == 1:
                    self.get_logger().info('🟢 BOTH FINGERS IN CONTACT!')
                
                if self.contact_stable_count > 3 and not self.gripper_stopped:
                    self.stop_gripper()
                    self.gripper_stopped = True
                return
        
        # Show individual finger contact
        if hasattr(self, 'left_contact_time') and current_time - self.left_contact_time < 0.1:
            if not (hasattr(self, 'right_contact_time') and current_time - self.right_contact_time < 0.2):
                self.get_logger().info('⚠️  Only LEFT finger', throttle_duration_sec=1.0)
        
        if hasattr(self, 'right_contact_time') and current_time - self.right_contact_time < 0.1:
            if not (hasattr(self, 'left_contact_time') and current_time - self.left_contact_time < 0.2):
                self.get_logger().info('⚠️  Only RIGHT finger', throttle_duration_sec=1.0)
        
        # Reset counter if contacts are too old
        if self.contact_stable_count > 0:
            if not (hasattr(self, 'left_contact_time') and hasattr(self, 'right_contact_time')):
                self.contact_stable_count = 0
            elif (current_time - self.left_contact_time > 0.3 or 
                  current_time - self.right_contact_time > 0.3):
                self.contact_stable_count = 0
    
    def stop_gripper(self):
        """Command gripper to hold current position"""
        self.get_logger().info(f'⏸️ STOPPING at {self.current_gripper_position:.4f}')
        
        traj = JointTrajectory()
        traj.joint_names = ['finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [self.current_gripper_position]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        
        traj.points = [point]
        self.gripper_pub.publish(traj)
        
        # Calculate relative pose between gripper and object at grasp moment
        try:
            # Get current gripper pose immediately
            self.grasp_gripper_transform = self.tf_buffer.lookup_transform(
                'world',
                'link6',
                rclpy.time.Time(), # Latest
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Get current box pose via service asynchronously
            get_req = GetEntityState.Request()
            get_req.name = 'red_box'
            get_req.reference_frame = 'world'
            
            future = self.get_state_client.call_async(get_req)
            future.add_done_callback(self.on_box_pose_received)
            
        except Exception as e:
            self.get_logger().error(f'Failed to initiate grasp calculation: {e}')

    def on_box_pose_received(self, future):
        try:
            result = future.result()
            if result and result.success:
                box_pose = result.state.pose
                
                # We need the gripper transform we captured earlier
                if not hasattr(self, 'grasp_gripper_transform'):
                    self.get_logger().error('Missing gripper transform for grasp calc')
                    return

                gripper_pos = self.grasp_gripper_transform.transform.translation
                
                # Store relative offset between gripper and box
                self.grasp_offset_x = box_pose.position.x - gripper_pos.x
                self.grasp_offset_y = box_pose.position.y - gripper_pos.y
                self.grasp_offset_z = box_pose.position.z - gripper_pos.z
                
                # Store box orientation
                self.grasp_orientation = box_pose.orientation
                
                self.get_logger().info(f'📍 Grasp offset: ({self.grasp_offset_x:.3f}, {self.grasp_offset_y:.3f}, {self.grasp_offset_z:.3f})')
                
                # Now attach
                self.object_attached = True
                self.get_logger().info('✅ Gripper STOPPED! Object ATTACHED!')
            else:
                self.get_logger().warn('Failed to get box pose (Service returned failure)')
                # Optional: Retry logic could be added here if needed, but risky for recursion loops
        except Exception as e:
            self.get_logger().error(f'Error in box pose callback: {e}')
    
    def update_attached_object(self):
        """Keep attached object following gripper"""
        # Manual attachment is disabled because we use gazebo_grasp_fix plugin.
        # This function is now a placeholder.
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GripperGraspController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
