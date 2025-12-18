#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, Int32, String
from erp42_msgs.msg import SerialFeedBack, ControlMessage
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray
from stanley_cone import Stanley
from tf_transformations import *
import numpy as np
import math as m
import threading
from pyproj import *


class PathHandler:
    def __init__(self, node, path_topic):
        node.create_subscription(
            Path, path_topic, self.callback_path, qos_profile_system_default
        )

        self.cx = []
        self.cy = []
        self.cyaw = []

    def callback_path(self, msg):
        self.cx, self.cy, self.cyaw = self.update_path(msg)

    def update_path(self, data):
        cx = []
        cy = []
        cyaw = []
        for p in data.poses:
            cx.append(p.pose.position.x)
            cy.append(p.pose.position.y)
            _, _, yaw = euler_from_quaternion(
                [
                    p.pose.orientation.x,
                    p.pose.orientation.y,
                    p.pose.orientation.z,
                    p.pose.orientation.w,
                ]
            )
            cyaw.append(yaw)
        return cx, cy, cyaw


class State:
    def __init__(self, node, odom_topic):
        node.create_subscription(
            Odometry, odom_topic, self.callback, qos_profile_system_default
        )
        node.create_subscription(
            SerialFeedBack,
            "erp42_feedback",
            self.callback_erp,
            qos_profile_system_default,
        )

        self.x = 0.0  # m
        self.y = 0.0  # m
        self.yaw = 0.0  # rad
        self.v = 0.0  # m/s

    def callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.yaw = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )

    def callback_erp(self, msg):
        self.v = msg.speed


# class SpeedSupporter():
#     def __init__(self, node):
#         # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
#         # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

#         # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
#         # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value


#         # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 50.0).value
#         # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 30.0).value

#         # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.001).value
#         # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.002).value

#         self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 40.0).value
#         self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 30.0).value

#         self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.02).value
#         self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.03).value

#     def func(self, x, a, b):
#         return a * (x - b)

#     def adaptSpeed(self,value,hdr,ctr,min_value,max_value):
#         hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
#         ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
#         err = hdr + ctr
#         res = np.clip(value + err, min_value, max_value)
#         return res


class SpeedSupporter:
    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(
        self, value, hdr, ctr, min_value, max_value, he_gain, ce_gain, he_thr, ce_thr
    ):
        hdr = self.func(abs(hdr), -he_gain, he_thr)
        ctr = self.func(abs(ctr), -ce_gain, ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)
        return res


class Drive:
    def __init__(self, node, state, path):
        self.pub = node.create_publisher(
            ControlMessage, "cmd_msg", qos_profile_system_default
        )

        self.st = Stanley()
        self.ss = SpeedSupporter()

        self.path = path
        self.state = state

        self.first_lap = True

    def publish_cmd(self):

        target_idx, error = self.st.calc_target_index(
            self.state, self.path.cx, self.path.cy
        )
        self.decision_first_lap(target_idx)

        if self.first_lap:  # first lap
            if self.decision_last_idx(target_idx):
                h_gain_curve = 0.8
                c_gain_curve = 0.5
                target_speed = 1.0

                steer, hdr, ctr = self.st.stanley_control(
                    self.state,
                    self.path.cyaw,
                    h_gain_curve,
                    c_gain_curve,
                    target_idx,
                    error,
                )
                adapted_speed = self.ss.adaptSpeed(
                    target_speed,
                    hdr,
                    ctr,
                    min_value=2,
                    max_value=4,
                    he_gain=50.0,
                    ce_gain=30.0,
                    he_thr=0.001,
                    ce_thr=0.002,
                )
                if self.state.v * 3.6 >= adapted_speed:
                    input_brake = (abs(self.state.v * 3.6 - adapted_speed) / 20.0) * 200
                else:
                    input_brake = 0

                print("close last idx", adapted_speed)

            else:
                if self.decision_straight(target_idx):
                    # h_gain_straight = 0.5
                    # c_gain_straight = 0.24

                    h_gain_straight = 0.6
                    c_gain_straight = 0.3
                    # target_speed = 10.0
                    target_speed = 12.0

                    steer, hdr, ctr = self.st.stanley_control(
                        self.state,
                        self.path.cyaw,
                        h_gain_straight,
                        c_gain_straight,
                        target_idx,
                        error,
                    )
                    adapted_speed = self.ss.adaptSpeed(
                        target_speed,
                        hdr,
                        ctr,
                        min_value=9,
                        max_value=12,
                        he_gain=40.0,
                        ce_gain=30.0,
                        he_thr=0.07,
                        ce_thr=0.05,
                    )
                    if self.state.v * 3.6 >= adapted_speed:
                        input_brake = (
                            abs(self.state.v * 3.6 - adapted_speed) / 20.0
                        ) * 200
                    else:
                        input_brake = 0

                    print("straight", adapted_speed)

                else:
                    h_gain_curve = 0.8
                    c_gain_curve = 0.5
                    target_speed = 7.0

                    steer, hdr, ctr = self.st.stanley_control(
                        self.state,
                        self.path.cyaw,
                        h_gain_curve,
                        c_gain_curve,
                        target_idx,
                        error,
                    )
                    adapted_speed = self.ss.adaptSpeed(
                        target_speed,
                        hdr,
                        ctr,
                        min_value=6,
                        max_value=7,
                        he_gain=50.0,
                        ce_gain=30.0,
                        he_thr=0.001,
                        ce_thr=0.002,
                    )
                    if self.state.v * 3.6 >= adapted_speed:
                        input_brake = (
                            abs(self.state.v * 3.6 - adapted_speed) / 20.0
                        ) * 200
                    else:
                        input_brake = 0

                    print("curve", adapted_speed)

        else:  # not first lap
            if self.decision_straight(target_idx):
                h_gain_straight = 0.6
                c_gain_straight = 0.4
                target_speed = 15.0

                steer, hdr, ctr = self.st.stanley_control(
                    self.state,
                    self.path.cyaw,
                    h_gain_straight,
                    c_gain_straight,
                    target_idx,
                    error,
                )
                adapted_speed = self.ss.adaptSpeed(
                    target_speed,
                    hdr,
                    ctr,
                    min_value=13,
                    max_value=15,
                    he_gain=40.0,
                    ce_gain=20.0,
                    he_thr=0.03,
                    ce_thr=0.05,
                )
                if self.state.v * 3.6 >= adapted_speed:
                    input_brake = (abs(self.state.v * 3.6 - adapted_speed) / 20.0) * 200
                else:
                    input_brake = 0

                print("straight", adapted_speed)

            else:
                h_gain_curve = 0.8
                c_gain_curve = 0.5
                target_speed = 8

                steer, hdr, ctr = self.st.stanley_control(
                    self.state,
                    self.path.cyaw,
                    h_gain_curve,
                    c_gain_curve,
                    target_idx,
                    error,
                )
                adapted_speed = self.ss.adaptSpeed(
                    target_speed,
                    hdr,
                    ctr,
                    min_value=5,
                    max_value=8,
                    he_gain=50.0,
                    ce_gain=30.0,
                    he_thr=0.001,
                    ce_thr=0.002,
                )
                if self.state.v * 3.6 >= adapted_speed:
                    input_brake = (abs(self.state.v * 3.6 - adapted_speed) / 20.0) * 200
                else:
                    input_brake = 0

                print("curve", adapted_speed)

        msg = ControlMessage()
        msg.speed = int(adapted_speed) * 10
        msg.steer = int(m.degrees((-1) * steer)*1e3)
        msg.gear = 2
        msg.brake = int(input_brake)

        self.pub.publish(msg)

    def decision_first_lap(self, target_idx):
        if (
            len(self.path.cyaw) >= 100 and target_idx <= 10
        ):  # 생성된 path가 10m 이상이고 target_idx가 10 이하일 떄, 즉 출발점 부근일 떼
            self.first_lap = False

    def decision_last_idx(self, target_idx):
        if (
            abs(len(self.path.cyaw) - target_idx) <= 10
        ):  # path의 마지막 노드랑 차랑 인덱스가 10개 이내일때 (즉 path의 마지막 노드랑 거리가 1m 이내일 때)
            return True

        else:
            return False

    def decision_straight(self, target_idx):
        yaw_list = []
        for i in range(target_idx - 5, target_idx + 30):
            try:
                yaw_list.append(self.path.cyaw[i])
            except IndexError:
                break
        mean = np.mean(np.abs(np.diff(yaw_list)))
        print(mean)
        if mean > 0.018:  # 1027 0.01 -> 0.0075 -> 0.015 -> 0.02
            return False
        else:
            return True


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("driving_node")
    state = State(node, "/odometry/navsat")
    path_tracking = PathHandler(node, "del_path")
    d = Drive(node, state, path_tracking)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(8)  # TODO hz 상승

    while rclpy.ok():
        try:
            d.publish_cmd()
        except Exception as ex:
            print(ex)
        rate.sleep()


if __name__ == "__main__":
    main()
