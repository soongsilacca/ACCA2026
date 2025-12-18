import rclpy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped
from tf_transformations import *
import math
import numpy as np
import time

from rclpy.qos import qos_profile_system_default
import os
import time
from stanley import Stanley
from erp42_msgs.msg import SerialFeedBack, ControlMessage


class SpeedSupporter:
    def __init__(self, node):
        # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value

        self.he_gain = node.declare_parameter("/speed_supporter/he_gain_stopline", 50.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain_stopline", 30.0).value

        self.he_thr = node.declare_parameter("/speed_supporter/he_thr_stopline",0.001).value
        self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr_stopline",0.002).value

    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self, value, hdr, ctr, min_value, max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)
        return res

class PID():
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain_stopline", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain_stopline", 0.85).value
       
        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0
        
        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
    def PIDControl(self, speed, desired_value):

        self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        dt = self.current - self.last
        self.last = self.current

        err = desired_value - speed
        # self.d_err = (err - self.p_err) / dt 
        self.p_err = err
        self.i_err += self.p_err * dt  * (0.0 if speed == 0 else 1.0)

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(self.speed, 7, 10))


class Stopline():
    def __init__(self, node):

        self.node = node

        self.target_idx = 0

        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)


        self.estop = 0
        self.target_speed = 10
        self.count = 0



    # def control_stop_line(self, odometry, path): modified 25.10.11
    def control_stop_line(self, odometry, path, mpc_steer, mpc_speed):
        stopline_finished = False

        msg = ControlMessage()
        steer, self.target_idx, hdr, ctr = self.st.stanley_control(odometry, path.cx, path.cy, path.cyaw, h_gain=0.5, c_gain=0.24)
        print(self.target_idx, len(path.cx))


        if self.target_idx >= len(path.cx) - 5: # distance가 0.5m 이내
            print(self.target_idx, len(path.cx), self.count)
            if self.count <= 30:
                self.estop = 1
                self.count += 1
            else:
                self.count = 0
                self.estop = 0
                stopline_finished = True
                self.target_idx = 0
            

        msg.steer = int(math.degrees((-1) * mpc_steer) * 1e3)
        msg.speed = int(mpc_speed) *10 
        msg.gear = 2
        msg.estop = self.estop

        return msg, stopline_finished
    
