#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF Global Initializer & Reset Monitor (ROS1)

This node:
1. Subscribes to /gps_pose and /imu to perform the initial alignment of the global EKF (via /set_pose_global).
2. Runs continuously to monitor GPS blackout (timeout) and recovery, and sudden jumps (spawn/teleport).
3. Re-initializes (resets) the global EKF and NDT pose when recovery with large drift or a spawn event is detected.
"""

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class EKFGlobalInitializer:
    def __init__(self):
        rospy.init_node('ekf_global_initializer', anonymous=True)
        
        # State variables
        self.initialized = False
        self.gps_pose_received = False
        self.imu_received = False
        
        self.initial_x = 0.0
        self.initial_y = 0.0
        
        # Continuous monitoring variables
        self.latest_q = None
        self.latest_yaw = 0.0
        self.latest_ekf_pose = None
        
        self.last_gps_x = 0.0
        self.last_gps_y = 0.0
        self.last_gps_time = rospy.Time(0)
        self.is_gps_active = False
        
        self.last_reset_time = rospy.Time(0)
        self.reset_cooldown = 3.0  # seconds
        
        # Parameters
        self.gps_timeout = rospy.get_param('~gps_timeout', 3.0)  # seconds
        self.jump_threshold = rospy.get_param('~jump_threshold', 30.0)  # meters (for spawn detection)
        self.recovery_drift_threshold = rospy.get_param('~recovery_drift_threshold', 30.0)  # meters (for blackout recovery)

        # Publisher for EKF Global initial/reset pose
        self.pub_set_pose = rospy.Publisher(
            '/set_pose_global', PoseWithCovarianceStamped, queue_size=1, latch=True
        )
        
        # Subscribers
        self.sub_gps_pose = rospy.Subscriber(
            '/gps_pose', PoseWithCovarianceStamped, self.gps_pose_callback
        )
        self.sub_imu = rospy.Subscriber(
            '/imu', Imu, self.imu_callback
        )
        self.sub_ekf = rospy.Subscriber(
            '/localization/kinematic_state', Odometry, self.ekf_callback
        )
        
        # Timer for timeout checking
        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_gps_timeout)
        
        rospy.loginfo(
            f"EKF Global Initializer & Reset Monitor Started.\n"
            f"  - GPS Timeout Threshold: {self.gps_timeout}s\n"
            f"  - Spawn Jump Threshold: {self.jump_threshold}m\n"
            f"  - Recovery Reset Drift Threshold: {self.recovery_drift_threshold}m\n"
            f"Waiting for /gps_pose and /imu to initialize..."
        )

    def imu_callback(self, msg):
        self.latest_q = msg.orientation
        _, _, yaw = euler_from_quaternion([self.latest_q.x, self.latest_q.y, self.latest_q.z, self.latest_q.w])
        self.latest_yaw = yaw
        
        self.imu_received = True
        self._try_initialize()

    def ekf_callback(self, msg):
        self.latest_ekf_pose = msg.pose.pose

    def gps_pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()

        # Handle time going backwards (e.g. bag loop or rewind)
        if now.to_sec() < self.last_gps_time.to_sec():
            rospy.logwarn("Time went backwards! Resetting EKF Global Initializer state.")
            self.initialized = False
            self.gps_pose_received = False
            self.is_gps_active = False
            self.latest_ekf_pose = None
            self.last_reset_time = rospy.Time(0)

        if not self.initialized:
            self.initial_x = x
            self.initial_y = y
            self.gps_pose_received = True
            self._try_initialize()
            if self.initialized:
                self.last_gps_x = x
                self.last_gps_y = y
                self.last_gps_time = now
                self.is_gps_active = True
            return

        # --- Continuous Monitoring ---
        
        # 1. Check recovery from GPS blackout
        if not self.is_gps_active:
            drift = 0.0
            if self.latest_ekf_pose is not None:
                ekf_x = self.latest_ekf_pose.position.x
                ekf_y = self.latest_ekf_pose.position.y
                drift = math.sqrt((x - ekf_x)**2 + (y - ekf_y)**2)
            
            rospy.loginfo(f"GPS recovered. Drift relative to EKF: {drift:.2f}m")
            
            # If drift is larger than threshold, force-reset the EKF/NDT pose
            if self.latest_ekf_pose is None or drift > self.recovery_drift_threshold:
                rospy.logwarn(
                    f"EKF has drifted significantly during blackout ({drift:.2f}m > {self.recovery_drift_threshold}m). "
                    f"Re-initializing EKF/NDT to recovered GPS pose."
                )
                self.reset_pose(x, y, now)
            else:
                rospy.loginfo("EKF drift is small. Resuming normal GPS fusion without reset.")
            
            self.is_gps_active = True

        # 2. Check sudden jump (spawn/teleport detection)
        else:
            dist = math.sqrt((x - self.last_gps_x)**2 + (y - self.last_gps_y)**2)
            if dist > self.jump_threshold:
                rospy.logwarn(
                    f"Sudden GPS jump detected (distance: {dist:.2f}m > {self.jump_threshold}m). "
                    f"Assuming vehicle spawned/teleported. Resetting EKF/NDT."
                )
                self.reset_pose(x, y, now)

        # Update last state
        self.last_gps_x = x
        self.last_gps_y = y
        self.last_gps_time = now

    def _try_initialize(self):
        if not (self.gps_pose_received and self.imu_received):
            return
            
        if self.initialized:
            return
            
        # Create and publish initial pose for EKF Global
        now = rospy.Time.now()
        self.reset_pose(self.initial_x, self.initial_y, now)
        self.initialized = True
        rospy.loginfo("EKF Global Initialized successfully. Monitoring active.")

    def reset_pose(self, x, y, stamp):
        now = rospy.Time.now()
        # Cooldown check to prevent resetting multiple times within a short duration
        if (now - self.last_reset_time).to_sec() < self.reset_cooldown:
            rospy.logdebug("Reset requested within cooldown period. Ignoring.")
            return

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        
        if self.latest_q is not None:
            pose_msg.pose.pose.orientation.x = self.latest_q.x
            pose_msg.pose.pose.orientation.y = self.latest_q.y
            pose_msg.pose.pose.orientation.z = self.latest_q.z
            pose_msg.pose.pose.orientation.w = self.latest_q.w
        else:
            pose_msg.pose.pose.orientation.w = 1.0
        
        cov = [0.0] * 36
        cov[0]  = 10.0   # X (increased initial uncertainty)
        cov[7]  = 10.0   # Y
        cov[14] = 999.0   # Z (무시)
        cov[21] = 999.0   # Roll (무시)
        cov[28] = 999.0   # Pitch (무시)
        cov[35] = 0.5   # Yaw (increased initial uncertainty)
        pose_msg.pose.covariance = cov
        
        self.pub_set_pose.publish(pose_msg)
        self.last_reset_time = now
        rospy.loginfo(
            f"Global EKF & NDT Reset Published → Position: ({x:.2f}, {y:.2f}), "
            f"Yaw: {math.degrees(self.latest_yaw):.1f} deg"
        )

    def check_gps_timeout(self, event):
        if not self.initialized:
            return
            
        if self.is_gps_active:
            now = rospy.Time.now()
            elapsed = (now - self.last_gps_time).to_sec()
            if elapsed > self.gps_timeout:
                self.is_gps_active = False
                rospy.logwarn(f"GPS blackout detected! No GPS pose received for {elapsed:.1f} seconds.")

if __name__ == '__main__':
    try:
        initializer = EKFGlobalInitializer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
