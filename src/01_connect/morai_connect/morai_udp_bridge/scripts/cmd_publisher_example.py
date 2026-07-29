#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_publisher_example.py

An example ROS node that publishes control commands to the '/cmd' topic
using the morai_msgs/CtrlCmd message type.
"""

import math
import rospy
from morai_msgs.msg import CtrlCmd


def main():
    # Initialize the ROS node
    rospy.init_node('cmd_publisher_example', anonymous=True)

    # Create a publisher for the '/cmd' topic
    cmd_pub = rospy.Publisher('/cmd', CtrlCmd, queue_size=10)

    # Publish rate: 10 Hz
    rate = rospy.Rate(10)

    rospy.loginfo("CMD Publisher Example started. Publishing to '/cmd'...")

    # CtrlCmd message initialization
    cmd_msg = CtrlCmd()

    cmd_msg.ctrl_mode = 2  # 1: Keyboard, 2: AutoMode
    cmd_msg.gear = 4       # 4: D (Drive)

    # Define command type: 1 for Throttle mode (accel, brake, steer)
    # 2: Velocity mode, 3: Acceleration mode
    cmd_msg.cmd_type = 1

    # Keep track of loop iteration to generate dynamic inputs
    count = 0

    while not rospy.is_shutdown():
        # Example dynamic control command:
        # - accel: 0.3 (constant forward acceleration command)
        # - brake: 0.0
        # - steer: sinusoidal steer input oscillating between -0.3 and 0.3 radians
        cmd_msg.accel = 1.0
        cmd_msg.brake = 0.0
        cmd_msg.steer = 0.3 * math.sin(count * 0.1)

        # Print current values to screen
        rospy.loginfo(
            f"Publishing -> Type: {cmd_msg.cmd_type}, Accel: {cmd_msg.accel:.2f}, "
            f"Brake: {cmd_msg.brake:.2f}, Steer: {cmd_msg.steer:.3f}"
        )

        # Publish the command
        cmd_pub.publish(cmd_msg)

        count += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
