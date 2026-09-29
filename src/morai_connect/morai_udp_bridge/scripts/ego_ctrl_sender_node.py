#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ego_ctrl_sender_node.py

ROS node to subscribe to control commands (/ctrl_cmd) and event commands
(/morai/event_cmd), and forward them to the MORAI simulator via UDP EgoCtrlCmd.
"""

import sys
import threading
from pathlib import Path

# Add the root directory to path for importing 'lib'
sys.path.append(str(Path(__file__).resolve().parents[2]))

import rospy
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import Bool, String
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
        self.estop_topic = rospy.get_param('~estop_topic', '/safety/estop')
        self.startup_accel_duration = max(0.0, float(rospy.get_param(
            '~startup_accel_duration_sec', 3.0)))
        self.startup_accel = min(max(float(rospy.get_param(
            '~startup_accel', 0.3)), 0.0), 1.0)
        self.episode_loaded_time = None
        self.estop_active = False
        self.lock = threading.Lock()

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
        self.episode_sub = rospy.Subscriber(
            '/morai/episode_status', String, self.episode_status_callback,
            queue_size=10)
        self.estop_sub = rospy.Subscriber(
            self.estop_topic, Bool, self.estop_callback, queue_size=1)

        rospy.loginfo(f"[EgoCtrlCmd] Subscribed to '{self.cmd_topic}'")
        rospy.loginfo(f"[EgoCtrlCmd] Safety override from '{self.estop_topic}'")
        rospy.loginfo(f"[EgoCtrlCmd] Forwarding to UDP {self.ip}:{self.port} at {self.send_rate}Hz")

    def ctrl_cmd_callback(self, msg):
        """Callback for incoming vehicle control commands (e.g. from pure pursuit or MPC)."""
        with self.lock:
            self.data.ctrl_mode = msg.ctrl_mode
            self.data.gear = msg.gear
            self.data.cmd_type = msg.cmd_type
            self.data.accel = msg.accel
            self.data.brake = msg.brake
            self.data.steer = msg.steer
            self.data.velocity = msg.velocity
            self.data.acceleration = msg.acceleration

    def episode_status_callback(self, msg):
        with self.lock:
            if ' LOADED ' in msg.data:
                self.episode_loaded_time = rospy.Time.now()
                rospy.loginfo(
                    '[EgoCtrlCmd] startup accel override: %.2f for %.1fs',
                    self.startup_accel, self.startup_accel_duration)
            elif ' LOAD_SENT ' in msg.data or ' LOAD_FAILED ' in msg.data:
                self.episode_loaded_time = None

    def estop_callback(self, msg):
        active = bool(msg.data)
        with self.lock:
            changed = active != self.estop_active
            self.estop_active = active
        if changed:
            if active:
                rospy.logwarn('[EgoCtrlCmd] E-STOP UDP override active: brake=1.0')
            else:
                rospy.loginfo('[EgoCtrlCmd] E-STOP UDP override released')

    def run(self):
        rate = rospy.Rate(self.send_rate)
        while not rospy.is_shutdown():
            with self.lock:
                startup_active = (
                    self.episode_loaded_time is not None and
                    (rospy.Time.now()-self.episode_loaded_time).to_sec()
                    < self.startup_accel_duration)
                if self.estop_active:
                    # Apply the stop at the final UDP boundary.  This remains
                    # authoritative even if another node publishes /ctrl_cmd
                    # directly and bypasses estop_gate's /ctrl_cmd_raw input.
                    original = (self.data.ctrl_mode, self.data.cmd_type,
                                self.data.velocity, self.data.acceleration,
                                self.data.accel, self.data.brake)
                    self.data.ctrl_mode = 2
                    self.data.cmd_type = 1
                    self.data.velocity = 0.0
                    self.data.acceleration = 0.0
                    self.data.accel = 0.0
                    self.data.brake = 1.0
                    self.sender.send(self.data)
                    (self.data.ctrl_mode, self.data.cmd_type,
                     self.data.velocity, self.data.acceleration,
                     self.data.accel, self.data.brake) = original
                elif startup_active:
                    original = (self.data.cmd_type, self.data.accel,
                                self.data.brake)
                    self.data.cmd_type = 1
                    self.data.accel = self.startup_accel
                    self.data.brake = 0.0
                    self.sender.send(self.data)
                    (self.data.cmd_type, self.data.accel,
                     self.data.brake) = original
                else:
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
