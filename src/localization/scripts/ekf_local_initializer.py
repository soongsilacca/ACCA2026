#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF Local Initializer (ROS1)

Subscribes to /imu to get initial robot orientation.
Sets initial pose of EKF Local via /set_pose_local.
This ensures odom frame is aligned with real-world heading instantly.
"""

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class EKFLocalInitializer:
    def __init__(self):
        rospy.init_node('ekf_local_initializer', anonymous=True)
        
        self.initialized = False
        
        # Publisher for EKF Local set_pose
        self.pub_set_pose = rospy.Publisher(
            '/set_pose_local', PoseWithCovarianceStamped, queue_size=1, latch=True
        )
        
        # Subscribe to /imu for initial orientation
        self.sub_imu = rospy.Subscriber(
            '/imu', Imu, self.imu_callback
        )
        
        rospy.loginfo("EKF Local Initializer Started. Waiting for /imu...")
        
    def imu_callback(self, msg):
        if self.initialized:
            return
            
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Create initial pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'odom'
        
        # Position is 0,0,0 (start point is odom origin)
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        
        pose_msg.pose.pose.orientation.x = q.x
        pose_msg.pose.pose.orientation.y = q.y
        pose_msg.pose.pose.orientation.z = q.z
        pose_msg.pose.pose.orientation.w = q.w
        
        # Covariance
        cov = [0.0] * 36
        cov[0]  = 0.01   # X (odom 원점에서 시작, 확실)
        cov[7]  = 0.01   # Y
        cov[14] = 999.0    # Z (무시)
        cov[21] = 999.0    # Roll (무시)
        cov[28] = 999.0    # Pitch (무시)
        cov[35] = 0.2    # Yaw (increased heading uncertainty due to IMU noise)
        pose_msg.pose.covariance = cov
        
        self.pub_set_pose.publish(pose_msg)
        
        rospy.loginfo(f"EKF Local Initializer: Initialized Local EKF → odom (0.0, 0.0), Yaw={math.degrees(yaw):.1f} deg")
        
        self.initialized = True
        # EKF 노드가 latched 메시지를 수신할 때까지 대기
        # (MORAI 실시간: EKF와 초기화 노드가 동시 기동하므로 race condition 방지)
        rospy.sleep(2.0)
        rospy.loginfo("EKF Local Initialization complete. Node shutting down.")
        self.sub_imu.unregister()
        rospy.signal_shutdown("EKF Local Initialized successfully.")

if __name__ == '__main__':
    try:
        initializer = EKFLocalInitializer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
