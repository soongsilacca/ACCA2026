#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf_transformations import *
import math as m
import numpy as np
from rclpy.qos import QoSProfile


def normalize_angle(angle):
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

class Rotate(Node):
    def __init__(self):
        super().__init__("rotate_yaw")
        qos_profile = QoSProfile(depth = 1)
        self.create_subscription(Imu, "imu/data", self.callback, qos_profile)
        self.create_subscription(TwistWithCovarianceStamped, "ublox_gps_node/fix_velocity", self.callback_gps_vel, qos_profile)
        self.create_subscription(String, "road_type", self.callback_shape, qos_profile)
        # self.create_subscription(Odometry, "odometry/navsat",self.callback_odom, qos_profile)

        self.pub = self.create_publisher(Imu, "imu/rotated", qos_profile)
        
        self.delta_yaw = self.declare_parameter("delta_yaw", 0.0).value
        self.gps_yaw = 0.
        self.v = 0.
        self.odom_yaw = 0.
        self.delta = 0.
        self.path_shape = "straight"
        self.forward = [None]*10 #큐 사이즈 10 고려해보기 -> 아래 decision_straight함수에서 하나씩 꺼내서 다 검사하기 때문에 연산 시간 때문에 조금 줄여야 할 수도 있음 / 심지어 imu는 400hz
        self.mean = 0.

    def decision_straight(self):
        sum = 0.
        for f in self.forward:
            if f == None:
                return False
            else:
                sum += f
        mean = sum / len(self.forward)
        for f in self.forward:
            dyaw = mean - f
            if abs(m.degrees(dyaw)) > 30:
                print("no!")
                return False
            else:
                pass
        self.mean = mean
        return True
        
    def callback_gps_vel(self,msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.v = m.sqrt(vx**2+vy**2)
        self.gps_yaw = m.atan2(vy,vx)
        self.forward.append(self.gps_yaw)
        del self.forward[0]

    # def callback_odom(self,msg):
    #     _,_,self.odom_yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

    def callback_shape(self,msg):
        self.path_shape = msg.data

    def callback(self,msg):
        _,_,yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        delta = self.gps_yaw - yaw

        #curve에서는 보정이 안 되고 있는 것 같은데 방안 생각해보기!
        if abs(delta) > m.radians(2) and abs(delta) < m.radians(90) and self.v > 0.30 :
            if self.decision_straight() and  self.delta == 0.0 :    # 콘트래킹 할 때 self.delta 가 불연속적으로 휙휙 바뀌는 상황 방지하기 위해 처음 한 번 보정값만 적용 -> self.delta == 0.0 일 때만 적용되도록 조건 추가
                self.delta = self.mean - yaw
        yaw_prev = yaw
        yaw = yaw + self.delta_yaw + self.delta
        x,y,z,w = quaternion_from_euler(0, 0, yaw)
        print("%.4f   %.4f  %.4f" %(m.degrees(yaw_prev),m.degrees(yaw),m.degrees(self.delta)))
        data = msg
        data.orientation.x = x
        data.orientation.y = y
        data.orientation.z = z
        data.orientation.w = w
        # scale = 10 ** m.degrees(delta)
        data.orientation_covariance = [0.00025000000000000005, 0.0, 0.0, 0.0, 0.00025000000000000005, 0.0, 0.0, 0.0, 0.00025000000000000005]
        self.pub.publish(data)
        print(self.v)

def main(args = None):
    rclpy.init(args=args)
    node = Rotate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
