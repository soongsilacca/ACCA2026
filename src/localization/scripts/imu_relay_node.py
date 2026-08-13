#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu

class ImuRelayNode:
    def __init__(self):
        rospy.init_node('imu_relay_node', anonymous=True)
        
        self.pub = rospy.Publisher('/imu_with_cov', Imu, queue_size=10)
        self.sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        rospy.loginfo("IMU Relay Node started. Listening to /imu and publishing to /imu_with_cov with covariance.")

    def imu_callback(self, msg):
        if msg.header.stamp.to_sec() == 0:
            msg.header.stamp = rospy.Time.now()

        # Add a small covariance to the orientation to prevent EKF NaNs
        cov = list(msg.orientation_covariance)
        cov[0] = 0.01  # roll variance
        cov[4] = 0.01  # pitch variance
        cov[8] = 0.01  # yaw variance
        msg.orientation_covariance = tuple(cov)

        # Angular velocity covariance
        ang_cov = list(msg.angular_velocity_covariance)
        ang_cov[0] = 0.01
        ang_cov[4] = 0.01
        ang_cov[8] = 0.01
        msg.angular_velocity_covariance = tuple(ang_cov)

        # Linear acceleration covariance
        lin_cov = list(msg.linear_acceleration_covariance)
        lin_cov[0] = 0.01
        lin_cov[4] = 0.01
        lin_cov[8] = 0.01
        msg.linear_acceleration_covariance = tuple(lin_cov)

        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        node = ImuRelayNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
