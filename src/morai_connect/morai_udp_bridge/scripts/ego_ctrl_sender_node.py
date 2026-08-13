#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ego_ctrl_sender_node.py

ROS node to subscribe to control commands (/ctrl_cmd) and event commands
(/morai/event_cmd), and forward them to the MORAI simulator via UDP EgoCtrlCmd.
"""

import sys
from pathlib import Path

# Add the root directory to path for importing 'lib'
sys.path.append(str(Path(__file__).resolve().parents[2]))

import rospy
from morai_msgs.msg import CtrlCmd
from lib.network.UDP import Sender
from lib.define.EgoCtrlCmd import EgoCtrlCmd


class MoraiEgoCtrlSender:
    def __init__(self):
        rospy.init_node('morai_ego_ctrl_sender', anonymous=False)

        # 1. Configurable Parameters
        self.ip = rospy.get_param('~ip', '127.0.0.1')
        self.port = rospy.get_param('~port', 9093)
        self.default_ctrl_mode = rospy.get_param('~ctrl_mode', 1)  # 1: Keyboard, 2: AutoMode
        self.default_gear = rospy.get_param('~gear', 4)            # 4: D (Drive)
        self.send_rate = rospy.get_param('~send_rate', 50.0)       # Hz
        self.cmd_topic = rospy.get_param('~cmd_topic', '/ctrl_cmd')

        # 2. Initialize UDP Sender and ctypes data structure
        self.sender = Sender(self.ip, self.port)
        self.data = EgoCtrlCmd()

        # Set default values
        self.data.ctrl_mode = self.default_ctrl_mode
        self.data.gear = self.default_gear
        self.data.cmd_type = 1  # 1: Throttle(accel,brake,steer), 2: Velocity(velocity,steer)
        self.data.accel = 0.0
        self.data.brake = 0.0
        self.data.steer = 0.0
        self.data.velocity = 0.0
        self.data.acceleration = 0.0

        # 3. Subscribers
        self.ctrl_sub = rospy.Subscriber(self.cmd_topic, CtrlCmd, self.ctrl_cmd_callback)

        rospy.loginfo(f"[EgoCtrlCmd] Subscribed to '{self.cmd_topic}'")
        rospy.loginfo(f"[EgoCtrlCmd] Forwarding to UDP {self.ip}:{self.port} at {self.send_rate}Hz")

    def ctrl_cmd_callback(self, msg):
        """Callback for incoming vehicle control commands (e.g. from pure pursuit or MPC)."""
        self.data.ctrl_mode = msg.ctrl_mode
        self.data.gear = msg.gear
        self.data.cmd_type = msg.cmd_type
        self.data.accel = msg.accel
        self.data.brake = msg.brake
        self.data.steer = msg.steer
        self.data.velocity = msg.velocity
        self.data.acceleration = msg.acceleration

    def run(self):
        rate = rospy.Rate(self.send_rate)
        while not rospy.is_shutdown():
            self.sender.send(self.data)
            rate.sleep()


def main():
    try:
        sender_node = MoraiEgoCtrlSender()
        sender_node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
