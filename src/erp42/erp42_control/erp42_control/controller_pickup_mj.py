import rclpy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped
from tf_transformations import *
import math
import numpy as np
from yolo_msg.msg import TrafficSign


from rclpy.qos import qos_profile_system_default
import os
import time
from stanley import Stanley
from erp42_msgs.msg import SerialFeedBack, ControlMessage
#############################################################################################
#info return value
# msg: ControlMessage
# abs_var: "abs1", "abs2", "abs3" 중 하나로 설정될 예정 -> 이 값은 무조건 state machine에 추가할것
# pickup_finished: bool, pickup이 완료되었는지 여부
#############################################################################################

class SpeedSupporter:
    def __init__(self, node):
        # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value

        self.he_gain = node.declare_parameter("/speed_supporter/he_gain_pickup", 50.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain_pickup", 30.0).value

        self.he_thr = node.declare_parameter("/speed_supporter/he_thr_pickup",0.001).value
        self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr_pickup",0.002).value

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
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain_pickup", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain_pickup", 0.85).value
       
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
        return int(np.clip(self.speed, 6, 8))


class Pickup():
    def __init__(self, node):

        self.node = node

        self.target_idx = 0
        self.pickup_finished = False

        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)


        self.yolo_sub = self.node.create_subscription(
            TrafficSign,
            "/traffic_sign",
            self.yolo_callback,
            10
        )



        self.estop = 0
        self.target_speed = 6
        self.count = 0
        self.queue = []
        self.abs_var = None  # abs_var A1, A2, A3으로 들어왔기 때문에, +3 해서 들어와 B1, B2, B3과 맞춰줌 

    def yolo_callback(self, msg):
        if msg.class_id in (1,2,3):
            self.queue.append(msg.class_id)
        if len(self.queue) > 10:
            self.queue.pop(0)
        # 3) 가장 많이 검출된 클래스가 6번 이상이면 abs_var 설정
        for label in (1,2,3):
            if self.queue.count(label) >= 6:
                idx = label    # "1", "2", "3"
                self.abs_var = int(label+3)
                self.node.get_logger().info(f"{self.abs_var}, detected")
                # 딱 한 번만 하고 끝내고 싶으면 return 추가


    # def control_pickup(self, odometry, path, mpc_steer , mpc_speed):

    #     msg = ControlMessage()

    #     steer, self.target_idx, hdr, ctr = self.st.stanley_control(odometry, path.cx, path.cy, path.cyaw, h_gain=0.6, c_gain=0.35)
    #     # self.get_logger().info(self.target_idx)

    #     if self.target_idx >= len(path.cx) - 15 : # distance가 0.5m 이내
    #         # self.get_logger().info(self.target_idx, len(path.cx), self.count)
    #         if self.count <= 30:
    #             self.estop = 1
    #             self.count += 1
    #             if self.abs_var is not None:
    #                 print(f"Pickup: {self.abs_var} detected, estop engaged.")
    #         else:
    #             self.estop = 0
    #             self.pickup_finished = True


    #     msg.steer = int(math.degrees((-1) * mpc_steer) * 1e3)
    #     msg.speed = int(mpc_speed * 3.6) * 10
    #     msg.gear = 2
    #     msg.estop = self.estop

    #     return msg, self.abs_var, self.pickup_finished

    def control_pickup(self, odometry, path):

        msg = ControlMessage()

        steer, self.target_idx, hdr, ctr = self.st.stanley_control(odometry, path.cx, path.cy, path.cyaw, h_gain=0.6, c_gain=0.35)
        # self.get_logger().info(self.target_idx)

        if self.target_idx >= len(path.cx) - 2 : # distance가 0.5m 이내
            # self.get_logger().info(self.target_idx, len(path.cx), self.count)
            if self.count <= 50:
                self.estop = 1
                self.count += 1
                if self.abs_var is not None:
                    print(f"Pickup: {self.abs_var} detected, estop engaged.") #TODO topic(String) 형태로 logging 하기 (bag file 저장용)
            else:
                self.estop = 0
                self.pickup_finished = True
            speed = 0

        elif self.target_idx >= len(path.cx) - 100: # pickup의 goal이랑 10m 이내부터 감속
            self.target_speed = (len(path.cx) - self.target_idx) / len(path.cx) * 15
            self.target_speed = int(np.clip(self.target_speed, 6, 8))
            adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, min_value=6, max_value=8)
            speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed)

        else:
            self.target_speed = 8
            adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, min_value=6, max_value=8)
            speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed)


        msg.steer = int(math.degrees((-1) * steer) * 1e3)
        msg.speed = int(speed) * 10
        msg.gear = 2
        msg.estop = self.estop

        return msg, self.abs_var, self.pickup_finished

