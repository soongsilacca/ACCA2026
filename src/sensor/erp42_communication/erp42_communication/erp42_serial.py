#!/usr/bin/env python3

import sys
import rclpy
import serial
import threading
import math as m
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool
from erp42_msgs.msg import SerialFeedBack, ControlMessage


exitThread = False


class Control:
    def __init__(self, port_num, node):
        qos_profile = QoSProfile(depth=10)

        # Packet Define

        s = 83
        t = 84
        x = 88
        aorm = 1
        estop = 0
        etx0 = 13
        etx1 = 10
        alive = 0

        self.data = bytearray(14)
        self.data[0] = s
        self.data[1] = t
        self.data[2] = x
        self.data[3] = aorm
        self.data[4] = estop
        self.data[12] = etx0
        self.data[13] = etx1

        self.alive = alive

        # Publisher

        self.feedback_pub = node.create_publisher(
            SerialFeedBack, "/erp42_feedback", qos_profile
        )

        # Subscriber

        self.control_sub = node.create_subscription(
            ControlMessage, "/cmd_msg", self.cmdCallback, qos_profile
        )

        self.feedback_msg = SerialFeedBack()
        self.cmd_msg = ControlMessage()

        self.showdata = True
        self.ser = serial.Serial(
            port=port_num,
            baudrate=115200,
        )

        # Threading
        thread = threading.Thread(target=self.receive_data)
        thread.daemon = True
        thread.start()

    def cmdCallback(self, msg):
        self.cmd_msg = msg

    def send_data(self, data=ControlMessage()):

        if not data.estop:
            # speed
            speed = data.speed
            # steer
            steer = data.steer * 71 / 1e3
            # 제한
            if steer > 1999:
                steer = 1999
            elif steer < -1999:
                steer = -1999

            if steer >= 0:
                self.data[8] = int(steer // 256)
                self.data[9] = int(steer % 256)
                # print(self.data[8], self.data[9])
            else:
                steer = -steer
                self.data[8] = int(255 - steer // 256)
                self.data[9] = int(255 - steer % 256)

            
            self.data[5] = data.gear  # gear
            self.data[6] = int(speed // 256)
            self.data[7] = int(speed % 256)
            print(self.data[6], self.data[7])
            self.data[10] = data.brake  # BREAK
        else:
            # speed
            speed = 0

            # steer
            steer = 0 * 71 
            if steer > 1999:
                steer = 1999
            if steer < -1999:
                steer = -1999

            if steer >= 0:
                self.data[8] = int(steer // 256)
                self.data[9] = int(steer % 256)
            else:
                steer = -steer
                self.data[8] = int(255 - steer // 256)
                self.data[9] = int(255 - steer % 256)

            self.data[5] = data.gear  # gear
            self.data[6] = int(speed // 256)
            self.data[7] = int(speed % 256)
            self.data[10] = 200  # BREAK

        self.data[11] = self.alive
        self.ser.write(self.data)

        self.alive = self.alive + 1
        if self.alive == 256:
            self.alive = 0

    def receive_data(self):

        # Read Serial

        line = []
        # print("out")
        while not exitThread:
            # print("in")
            try:
                # print("try")
                for i in self.ser.read():
                    # print(i)
                    line.append(i)
                    if i == 83:
                        del line[:]
                        line.append(i)
                    elif i == 10 and line[-1] == 10 and len(line) == 18:
                        self.handle_data(line)
                        del line[:]
                        break
                    if len(line) >= 18:
                        del line[:]
                        break

            except Exception as ex:
                print(ex)

    def handle_data(self, line):

        # Transfer packet data to aorm / gear / speed(m/s) / steer(rad)
        print(line)
        # aorm - 0: M / 1: A
        # feedback_aorm = line[4]
        feedback_aorm = line[3]

        # estop
        feedback_estop = line[4]

        # gear - 0: B / 1: N / 2: D
        feedback_gear = line[5]

        # speed (m/s)
        feedback_KPH = (line[6] + line[7] * 256) / 10
        feedback_speed = self.kph2mps(value=feedback_KPH)

        # steer (RAD)
        feedback_DEG = line[8] + line[9] * 256

        if feedback_DEG >= 23768:
            feedback_DEG -= 65536

        if feedback_DEG != 0:
            feedback_DEG /= 71.0

        feedback_steer = m.radians(feedback_DEG)

        # Brake
        feedback_brake = line[10]

        # Encoder
        self.feedback_encoder = (
            line[11]
            + line[12] * 256
            + line[13] * 256 * 256
            + line[14] * 256 * 256 * 256
        )

        if self.feedback_encoder >= 2147483648:
            self.feedback_encoder -= 4294967296

        data = SerialFeedBack()

        data.mora = feedback_aorm
        data.estop = feedback_estop
        data.gear = feedback_gear
        data.speed = feedback_speed
        data.steer = feedback_steer
        data.brake = feedback_brake
        data.encoder = self.feedback_encoder
        data.alive = self.alive

        self.feedback_msg = data

        # print(data)
        self.feedback_pub.publish(self.feedback_msg)

        # if self.showdata is True:
        #     self.get_logger().info(str(self.feedback_msg))

    """ Util """

    def kph2mps(self, value):
        return value * 0.277778

    def mps2kph(self, value):
        return value * 3.6


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("erp42_serial")

    port_param = node.declare_parameter(
        "/erp42_serial/erp_port", "/dev/ttyUSB1"
    )  # launch파일에 파라미터 정의해서 가져오기 안 됨. 다 정상적으로 되면 마지막에 이 부분 공부해보기

    port = port_param.value
    print(port)
    print("connected successfully!!")

    control = Control(port_num=port, node=node)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(20.0)

    while rclpy.ok():
        control.send_data(data=control.cmd_msg)
        rate.sleep()


if __name__ == "__main__":
    main()
