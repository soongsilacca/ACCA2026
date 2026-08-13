#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import os
import yaml
import rospkg
import numpy as np

from collections import deque

from morai_msgs.msg import TrackingInfo, MPCWeight
from nav_msgs.msg import Odometry



class AdaptiveWeightNode:

    def __init__(self):

        package_path = rospkg.RosPack().get_path('control')
        yaml_candidates = [
            os.path.join(package_path, 'config', 'config.yaml'),
            os.path.join(package_path, 'src', 'config.yaml'),
            os.path.join(package_path, 'parameter', 'config.yaml'),
        ]
        yaml_path = next((path for path in yaml_candidates if os.path.exists(path)), None)
        if yaml_path is None:
            raise FileNotFoundError("MPC config.yaml not found in: %s" % yaml_candidates)

        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        self.__dict__.update(config)

        self.e_ct = 0.0
        self.e_yaw = 0.0
        self.curve_gain = 0.0

        self.steer = 0.0
        self.prev_steer = 0.0

        self.steer_rate = 0.0

        self.q_ct = self.q_ct_base
        self.q_yaw = self.q_yaw_base
        self.r_steer = self.r_steer_base

        self.cte_history = deque(maxlen=self.history_size)
        self.yaw_history = deque(maxlen=self.history_size)
        self.steer_history = deque(maxlen=self.history_size)
        self.steer_rate_history = deque(maxlen=self.history_size)

        self.prev_cte = 0.0
        self.prev_yaw = 0.0

        self.cte_dot = 0.0
        self.yaw_dot = 0.0

        self.cte_dot_history = deque(maxlen=self.history_size)
        self.yaw_dot_history = deque(maxlen=self.history_size)

        self.current_v = 0.0

        self.sub_tracking = rospy.Subscriber(
            "/tracking_info",
            TrackingInfo,
            self.trackingCallback,
            queue_size=10
        )

        self.sub_odom = rospy.Subscriber('/localization/kinematic_state', Odometry, self.odom_callback, queue_size = 10)

        self.pub_weight = rospy.Publisher(
            "/mpc_weight",
            MPCWeight,
            queue_size=10
        )

        self.timer = rospy.Timer(
            rospy.Duration(0.1),
            self.timerCallback
        )

        self.is_first_callback = True

        rospy.loginfo("Adaptive Weight Node Started")

    def odom_callback(self, msg):

        self.current_v = msg.twist.twist.linear.x

    def trackingCallback(self, msg):

        if self.is_first_callback:
            self.prev_cte = self.e_ct
            self.prev_yaw = self.e_yaw
            self.prev_steer = self.steer
            self.is_first_callback = False

        self.e_ct = msg.cross_track_error
        self.e_yaw = msg.heading_error

        self.curve_gain = msg.curvature_gain

        self.steer = msg.steering

        self.cte_dot = (self.e_ct - self.prev_cte) / self.dt

        self.yaw_dot = (self.e_yaw - self.prev_yaw) / self.dt

        self.prev_cte = self.e_ct
        self.prev_yaw = self.e_yaw

        self.steer_rate = (self.steer - self.prev_steer) / self.dt

        self.prev_steer = self.steer

        self.cte_history.append(self.e_ct)
        self.yaw_history.append(self.e_yaw)

        self.cte_dot_history.append(self.cte_dot)
        self.yaw_dot_history.append(self.yaw_dot)

        self.steer_history.append(self.steer)
        self.steer_rate_history.append(self.steer_rate)

    def calcRMS(self, data):

        if len(data) == 0:
            return 0.0

        data = np.array(data)

        return np.sqrt(np.mean(np.square(data)))

    def updateWeight(self):
        cte_rms = self.calcRMS(self.cte_history)
        yaw_rms = self.calcRMS(self.yaw_history)

        steer_rate_rms = self.calcRMS(self.steer_rate_history)

        cte_dot_rms = self.calcRMS(self.cte_dot_history)

        yaw_dot_rms = self.calcRMS(self.yaw_dot_history)

        v = abs(self.current_v)

        # Lateral acceleration grows approximately with v^2 * steer / L.
        # Schedule both lateral-error damping and absolute steering cost with
        # the same continuous squared-speed factor; no speed-mode switch.
        speed_reference = max(
            float(getattr(self, 'speed_weight_reference_kph', 35.0)) / 3.6,
            0.1
        )
        speed_factor = (v / speed_reference) ** 2

        gain_ct = (
                1.0
                + self.k_cte * np.tanh(cte_rms)
                + self.k_cte_dot * np.tanh(cte_dot_rms)
                + self.k_curve * self.curve_gain
            )

        speed_q_damping = 1.0 + self.k_speed_q * speed_factor
        Qct_target = self.q_ct_base * gain_ct / speed_q_damping

        gain_yaw = (
                1.0
                + self.k_yaw * np.tanh(yaw_rms)
                + self.k_yaw_dot * np.tanh(yaw_dot_rms)
                + self.k_curve * self.curve_gain
            )
            
        Qyaw_target = self.q_yaw_base * gain_yaw / speed_q_damping

        gain_r = (
                1.0
                + self.k_steer * np.tanh(steer_rate_rms)
                + self.k_speed_r * speed_factor
                - self.k_curve_r * self.curve_gain
            )

        gain_r = max(0.1, gain_r)

        Rsteer_target = self.r_steer_base * gain_r

        self.q_ct = (
            self.alpha * self.q_ct
            + (1-self.alpha) * Qct_target
        )

        self.q_yaw = (
            self.alpha * self.q_yaw
            + (1-self.alpha) * Qyaw_target
        )

        self.r_steer = (
            self.alpha * self.r_steer
            + (1-self.alpha) * Rsteer_target
        )

        if abs(self.curve_gain) < self.straight_throttle:
            q_ct_max = 3.0
        elif abs(self.curve_gain) < self.endcurve_throttle:
            q_ct_max = 4.0
        elif abs(self.curve_gain) < self.curve_throttle:
            q_ct_max = 5.0
        else:
            q_ct_max = self.q_ct_max

        self.q_ct = np.clip(
            self.q_ct,
            self.q_ct_min,
            q_ct_max
        )

        self.q_yaw = np.clip(
            self.q_yaw,
            self.q_yaw_min,
            self.q_yaw_max
        )

        self.r_steer = np.clip(
            self.r_steer,
            self.r_steer_min,
            self.r_steer_max
        )

        rospy.loginfo_throttle(
            1.0,
            "Adaptive speed schedule: v=%.1f km/h factor=%.3f "
            "Qct=%.2f Qyaw=%.2f Rsteer=%.2f",
            v * 3.6,
            speed_factor,
            self.q_ct,
            self.q_yaw,
            self.r_steer
        )

    def publishWeight(self):

        msg = MPCWeight()

        msg.q_ct = self.q_ct
        msg.q_v = self.q_v_base
        msg.q_yaw = self.q_yaw

        msg.r_accel = self.r_accel_base
        msg.r_steer = self.r_steer

        self.pub_weight.publish(msg)

    def timerCallback(self, event):

        self.updateWeight()

        self.publishWeight()


if __name__ == '__main__':
    try:
        rospy.init_node('adaptive_weight')
        AdaptiveWeightNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
