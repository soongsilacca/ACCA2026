#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Safety gate: turns /safety/estop_trigger into an actual brake command.

The judgement node only reports danger. This node is the part that stops the
car, and it is the last thing on the path to the simulator so it works even
when everything upstream is dead.

Modes
-----
override_only (default)
    Publishes nothing while safe, so keyboard driving stays in the driver's
    hands. On E-STOP it takes over and brakes. Use this for manual testing.

gate
    Relays /ctrl_cmd_raw to /Ctrl_cmd continuously and substitutes a brake
    command while E-STOP is latched. Use this once MPC feeds ctrl_cmd_raw.

Latch behaviour
---------------
Braking holds until the trigger clears AND the vehicle has been below
`release_speed` for `release_hold_sec`. Releasing on the trigger alone would
let the car roll away while still moving.
"""

import threading

import rospy
from std_msgs.msg import Bool, Float32, String

from morai_msgs.msg import CtrlCmd

try:
    from morai_msgs.msg import EgoVehicleStatus
    HAS_STATUS = True
except ImportError:
    HAS_STATUS = False

KPH_TO_MPS = 1.0 / 3.6

LONG_CMD_THROTTLE = 1      # accel / brake, per competition rules
CTRL_MODE_AUTO = 2         # external control
GEAR_DRIVE = 4


class SafetyGate(object):

    def __init__(self):
        self.mode = rospy.get_param("~mode", "override_only")
        self.publish_hz = rospy.get_param("~publish_hz", 30.0)
        self.brake_value = rospy.get_param("~brake_value", 1.0)
        self.ctrl_mode = rospy.get_param("~ctrl_mode", CTRL_MODE_AUTO)
        self.gear = rospy.get_param("~gear", GEAR_DRIVE)
        self.cmd_type = rospy.get_param("~cmd_type", LONG_CMD_THROTTLE)

        self.release_speed = rospy.get_param("~release_speed", 0.2)
        self.release_hold_sec = rospy.get_param("~release_hold_sec", 1.0)
        self.trigger_timeout = rospy.get_param("~trigger_timeout", 0.5)
        self.raw_timeout = rospy.get_param("~raw_timeout", 0.5)

        self.lock = threading.Lock()
        self.trigger = False
        self.last_trigger_time = None
        self.raw_cmd = None
        self.last_raw_time = None
        self.speed = 0.0

        self.braking = False
        self.stopped_since = None
        self.cause = "STANDBY"

        self.pub_cmd = rospy.Publisher(
            rospy.get_param("~ctrl_cmd_topic", "/ctrl_cmd"), CtrlCmd, queue_size=1)
        self.pub_state = rospy.Publisher("/safety/gate_state", String, queue_size=1)

        rospy.Subscriber("/safety/estop_trigger", Bool, self.trigger_cb, queue_size=1)

        if self.mode == "gate":
            rospy.Subscriber(rospy.get_param("~raw_cmd_topic", "/ctrl_cmd_raw"),
                             CtrlCmd, self.raw_cb, queue_size=1)

        if HAS_STATUS:
            rospy.Subscriber(rospy.get_param("~vehicle_status_topic",
                                             "/morai/ego_vehicle_status"),
                             EgoVehicleStatus, self.status_cb, queue_size=1)
        else:
            rospy.Subscriber("/safety/ego_speed_mps", Float32,
                             self.speed_fallback_cb, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / self.publish_hz), self.tick)
        rospy.loginfo("safety_gate up | mode=%s brake=%.2f", self.mode, self.brake_value)

    # ------------------------------------------------------------------- I/O

    def trigger_cb(self, msg):
        with self.lock:
            self.trigger = bool(msg.data)
            self.last_trigger_time = rospy.Time.now()

    def raw_cb(self, msg):
        with self.lock:
            self.raw_cmd = msg
            self.last_raw_time = rospy.Time.now()

    def status_cb(self, msg):
        with self.lock:
            self.speed = abs(float(msg.velocity.x)) * KPH_TO_MPS

    def speed_fallback_cb(self, msg):
        with self.lock:
            self.speed = abs(float(msg.data))

    # -------------------------------------------------------------- commands

    def brake_cmd(self):
        cmd = CtrlCmd()
        cmd.ctrl_mode = self.ctrl_mode
        cmd.gear = self.gear
        cmd.cmd_type = self.cmd_type
        cmd.velocity = 0.0
        cmd.acceleration = 0.0
        cmd.accel = 0.0
        cmd.brake = self.brake_value
        cmd.steer = 0.0
        self.pub_cmd.publish(cmd)

    def straight_cmd(self):
        cmd = CtrlCmd()
        cmd.ctrl_mode = self.ctrl_mode
        cmd.gear = self.gear
        cmd.cmd_type = self.cmd_type
        cmd.velocity = 0.0
        cmd.acceleration = 0.0
        cmd.accel = 0.5
        cmd.brake = 0.0
        cmd.steer = 0.0
        self.pub_cmd.publish(cmd)

    # ------------------------------------------------------------------ loop

    def tick(self, _event):
        now = rospy.Time.now()
        with self.lock:
            trigger = self.trigger
            last_trigger = self.last_trigger_time
            raw, last_raw = self.raw_cmd, self.last_raw_time
            speed = self.speed

        # The judgement node going silent is itself a fault, but only once it
        # has spoken at least one -- otherwise the gate would brake on startup
        # before the perception node is up.
        if last_trigger is not None and (now - last_trigger).to_sec() > self.trigger_timeout:
            trigger = True
            cause = "TRIGGER_TIMEOUT"
        else:
            cause = "ESTOP" if trigger else ""

        if trigger:
            self.braking = True
            self.stopped_since = None
            self.cause = cause
        elif self.braking:
            # hold the brake until the car has actually come to rest
            if speed <= self.release_speed:
                if self.stopped_since is None:
                    self.stopped_since = now
                elif (now - self.stopped_since).to_sec() >= self.release_hold_sec:
                    self.braking = False
                    self.cause = "RELEASED"
                    rospy.loginfo("gate released")
            else:
                self.stopped_since = None
                self.cause = "STOPPING_%.1fmps" % speed

        if self.braking:
            self.brake_cmd()
            self.pub_state.publish(String(data=self.cause))
        else:
            self.straight_cmd()

        if self.mode == "gate":
            if raw is not None and (now - last_raw).to_sec() <= self.raw_timeout:
                self.pub_cmd.publish(raw)
                self.pub_state.publish(String(data="PASSTHROUGH"))
            else:
                # upstream controller is silent: brake rather than coast
                self.brake_cmd()
                self.pub_state.publish(String(data="RAW_TIMEOUT"))
        else:
            # override_only: stay off the bus so manual driving is untouched
            self.pub_state.publish(String(data="IDLE"))


if __name__ == "__main__":
    rospy.init_node("safety_gate")
    SafetyGate()
    rospy.spin()
