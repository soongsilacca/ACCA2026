#!/usr/bin/env python3

import rclpy
import numpy as np
import math as m
from erp42_msgs.msg import StanleyError,SerialFeedBack
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32



"""

Export module. Stanley Control Class.
Input: (state(class: State), [cx], [cy], [cyaw], last_target_idx)
Output: steer

"""

class Stanley(Node):
    def __init__(self):
        super().__init__("stanley")

        # self.__L = 1.040  # [m] Wheel base of vehicle
        self.__L = 2.240  # [m] Wheel base of vehicle
        # self.__k = self.declare_parameter("/stanley_controller/c_gain", 0.8).value
        # self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.03).value
        # self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.06).value
        # self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.5).value


        self.__hdr = 0.0    #heading error
        self.__ctr = 0.0    #crosstrack error
        
        self.k_v = 0.5
        
    
    def setHdrRatio(self, value):
        self.__hdr_ratio = value
        return self.__hdr_ratio
    
    # def stanley_control(self, state, cx, cy, cyaw, last_target_idx, reverse=False):
    def stanley_control(self, state, cyaw, h_gain, c_gain, current_target_idx, error_front_axle, reverse=False):

        theta_e = (self.normalize_angle(
            cyaw[current_target_idx] - (state.yaw + (np.pi if reverse else 0.)))) * h_gain
        
        
        theta_d = np.arctan2(c_gain * error_front_axle,
                           (self.k_v + state.v)) * (-1.0 if reverse else 1.0)
        

        # Field
        self.__hdr = theta_e
        self.__ctr = theta_d

        # Steering control
        delta = theta_e + theta_d

        delta = np.clip(delta, m.radians((-1) * 28), m.radians(28))

        return delta, self.__hdr, self.__ctr
    
    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def calc_target_index(self, state, cx, cy, reverse=False):
        """
        Compute index in the trajectory list of the target.
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position (앞 축 구하는 것 수정 24.10.17)
        fx = state.x + self.__L * \
            np.cos(state.yaw) / 2.0 * (-1.0 if reverse else 1.0)
        fy = state.y + self.__L * \
            np.sin(state.yaw) / 2.0 * (-1.0 if reverse else 1.0)
        
        # fx = state.x
        # fy = state.y 


        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]

        d = np.hypot(dx, dy)
        target_idx = int(np.argmin(d))

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -np.sin(state.yaw + np.pi / 2)]

        error_front_axle = np.dot(
            [dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle
    