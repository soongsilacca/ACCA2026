#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from erp42_msgs.msg import ControlMessage
from erp42_msgs.msg import SerialFeedBack
from std_msgs.msg import *
import math as m
import numpy as np
import matplotlib.pyplot as plt
from time import time
from rclpy.qos import QoSProfile
import threading


class PID:
    def __init__(self, node):
        self.node = node
        qos_profile = QoSProfile(depth=1)

        self.p_gain = 2.07

        #  'Ku': 4.6, 'Tu': 2.9223529411764706

        self.i_gain = 0.85
        # self.i_gain = 0.0

        self.p_err = 0.0
        self.i_err = 0.0

        self.speed = 0
        self.final_speed = 0.0
        self.steer = 0.0
        self.desired_value = 0.0

        self.init_time = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        self.time = []

        self.max_v = 0.0
        self.overshoot = 0.0
        self.overshoot_percent = 0.0
        self.rising_time = 0.0
        self.rising_time_10 = 0.0
        self.rising_time_90 = 0.0
        self.settling_time = 50.0
        self.steady_state_err = 0.0
        self.flag = True

        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        self.dt = 0.0
        node.create_subscription(
            SerialFeedBack, "/erp42_feedback", self.callback_erp, qos_profile
        )
        self.pub = node.create_publisher(ControlMessage, "/cmd_msg", qos_profile)

    def PIDControl(self, desired_value):  # desired_value = m/s
        self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (
            self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        self.time.append(self.current - self.init_time)

        self.dt = self.current - self.last
        err = desired_value - self.speed

        self.p_err = err
        self.i_err += self.p_err * self.dt * (0.0 if self.speed == 0 else 1.0)

        ############ 수정 2025.08.04 ##############
        clip_value = 5.0  # Limit the integral error to prevent windup
        if self.i_err > clip_value:
            self.i_err = clip_value
        elif self.i_err < -1*clip_value:
            self.i_err = -1*clip_value
        ############ 수정 2025.08.04 ##############

        self.last = self.current

        speed = self.speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)

        return int(np.clip(speed, 0, 25))

    def cmd_pub(self):
        self.desired_value = 6.0
        self.final_speed = self.PIDControl(self.desired_value)

        msg = ControlMessage()
        msg.speed = self.final_speed * 10
        # msg.speed = int(self.desired_value * 10)
        msg.steer = int(m.degrees((-1) * self.steer))
        msg.gear = 2
        msg.brake = 0

        self.pub.publish(msg)

    def make_txt(self, tv, flag):
        path = "/home/acca/all/pid_test/p:{}, i:{}, v:{}_0808_48.txt".format(
            self.p_gain, self.i_gain, self.desired_value
        )
        # path = "/home/ps/6_pid_test/single_pid.txt"
        with open(path, "a") as f:
            if flag:
                data = "{}, {}\n".format(tv[0], tv[1])
                f.write(data)
            else:
                f.write(
                    "\novershoot:{}, rising_time:{}, settling_time:{}, ss_err:{}\n".format(
                        self.overshoot,
                        self.rising_time,
                        self.settling_time,
                        self.steady_state_err,
                    )
                )
                f.close()

    def callback_erp(self, msg):
        self.speed = msg.speed * 3.6  # km/h

    def kph2mps(value):
        return value * 0.277778

    def mps2kph(value):
        return value * 3.6


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("pid_tunning")

    pid = PID(node)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(20)
    target_v = []
    current_v = []

    count = 0
    flag = True

    while rclpy.ok():
        try:
            target_v.append(pid.final_speed)
            current_v.append(pid.speed)
            pid.cmd_pub()
            pid.make_txt([pid.time[-1], current_v[-1]], flag)
            print(pid.time[-1], pid.final_speed)

            if current_v[-1] >= (pid.desired_value * 0.1) and pid.rising_time_10 == 0.0:
                pid.rising_time_10 = pid.time[-1]

            if current_v[-1] >= (pid.desired_value * 0.9) and pid.rising_time_90 == 0.0:
                pid.rising_time_90 = pid.time[-1]
                pid.rising_time = pid.rising_time_90 - pid.rising_time_10

            if abs(current_v[-1] - pid.desired_value) / pid.desired_value <= 0.1:
                count += 1
                if count == 120 and pid.settling_time == 50.0:
                    pid.settling_time = pid.time[-1]
            else:
                count = 0

            if pid.time[-1] >= pid.settling_time:

                pid.max_v = max(current_v)
                pid.overshoot = pid.max_v - pid.desired_value
                pid.overshoot_percent = (pid.overshoot / pid.desired_value) * 100
                pid.steady_state_err = pid.speed - pid.desired_value
                flag = False
                pid.make_txt([pid.time[-1], current_v[-1]], flag)
                plt.plot(pid.time, current_v, c="b")
                plt.plot(pid.time, [pid.desired_value] * len(pid.time), c="g")
                plt.grid(True)
                plt.xlabel("time")
                plt.axis("equal")
                plt.show()

                break

        except Exception as ex:
            print(ex)
        rate.sleep()


if __name__ == "__main__":
    main()


# https://www.ni.com/en/shop/labview/pid-theory-explained.html
