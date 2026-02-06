#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class TestPickAndLift(Node):
    def __init__(self):
        super().__init__('test_pick_and_lift')
        
        self.arm_pub = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory, 
            '/gripper_controller/joint_trajectory', 
            10
        )
        
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_callback, 
            10
        )
        
        self.current_joints = {}
        self.received_state = False
        
        self.arm_joints = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.gripper_joint = 'finger_joint' # Mimic joints follow this
        
    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.current_joints[name] = msg.position[i]
        self.received_state = True

    def close_gripper(self):
        print("Closing Gripper...")
        msg = JointTrajectory()
        msg.joint_names = [self.gripper_joint]
        point = JointTrajectoryPoint()
        point.positions = [0.8] # Close (max is roughly 0.8)
        point.time_from_start = Duration(sec=2, nanosec=0)
        msg.points = [point]
        self.gripper_pub.publish(msg)
        
    def lift_arm(self):
        if not self.received_state:
            print("No joint state received yet!")
            return
            
        print("Lifting Arm...")
        msg = JointTrajectory()
        msg.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        positions = []
        for name in self.arm_joints:
            pos = self.current_joints.get(name, 0.0)
            if name == 'joint1': # Shoulder
                pos -= 0.5 # Lift up (check sign direction, usually negative is back/up for indy7 depending on zero)
                # Actually indy joint1 zero is vertical? No, usually horizontal.
                # Let's try changing joint1 by -0.3.
            positions.append(pos)
            
        point.positions = positions
        point.time_from_start = Duration(sec=4, nanosec=0)
        msg.points = [point]
        self.arm_pub.publish(msg)

def main():
    rclpy.init()
    node = TestPickAndLift()
    
    # Wait for connection
    time.sleep(1)
    
    # Wait for joint states
    print("Waiting for joint states...")
    while not node.received_state:
        rclpy.spin_once(node)
        time.sleep(0.1)
        
    # 1. Close Gripper
    node.close_gripper()
    
    # Wait for grasp and plugin attachment
    print("Waiting 3 seconds for grasp...")
    for _ in range(30):
        rclpy.spin_once(node)
        time.sleep(0.1)
    
    # 2. Lift Arm
    node.lift_arm()
    
    print("Lift command sent. Spinning...")
    rclpy.spin(node)

if __name__ == '__main__':
    main()
