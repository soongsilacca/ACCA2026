#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, Quaternion
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math as m
import numpy as np
from rclpy.qos import QoSProfile

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return qx, qy, qz, qw


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
        qos_profile = QoSProfile(depth=1)
        self.create_subscription(Imu, "imu/data", self.callback, qos_profile)
        self.create_subscription(
            TwistWithCovarianceStamped,
            "ublox_gps_node/fix_velocity",
            self.callback_gps_vel,
            qos_profile,
        )
        self.create_subscription(NavSatFix, "ublox_gps_node/fix",self.callback_cov, qos_profile)
        # self.create_subscription(String, "road_type", self.callback_shape, qos_profile)
        # self.create_subscription(Odometry, "odometry/navsat",self.callback_odom, qos_profile)

        self.pub = self.create_publisher(Imu, "imu/rotated", qos_profile)

        self.pub_gps = self.create_publisher(Quaternion, "mean", qos_profile)

        self.delta_yaw = self.declare_parameter("delta_yaw", 0.0).value
        self.gps_yaw = 0.0
        self.v = 0.0
        self.odom_yaw = 0.0
        self.delta = 0.0
        self.path_shape = "straight"
        self.forward = [
            None
        ] * 10  # 큐 사이즈 10 고려해보기 -> 아래 decision_straight함수에서 하나씩 꺼내서 다 검사하기 때문에 연산 시간 때문에 조금 줄여야 할 수도 있음 / 심지어 imu는 400hz
        self.mean = 0.0
        self.cov = 0.0

    def decision_straight(self):
        sum = 0.0
        for f in self.forward:
            if f == None:
                return False
            else:
                sum += f
        mean = sum / len(self.forward)
        for f in self.forward:
            dyaw = mean - f
            if abs(m.degrees(dyaw)) > 2:
                print("no!")
                return False
            else:
                pass
        self.mean = mean
        return True
    def callback_cov(self, msg):
        self.cov = msg.position_covariance[0]

    def callback_gps_vel(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.v = m.sqrt(vx**2 + vy**2)
        print(self.v)
        self.gps_yaw = m.atan2(vy, vx)
        self.forward.append(self.gps_yaw)
        del self.forward[0]

    # def callback_odom(self,msg):
    #     _,_,self.odom_yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

    # def callback_shape(self, msg):
    #     self.path_shape = msg.data

    def callback(self, msg):
        _, _, yaw = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )
        delta = normalize_angle(self.gps_yaw - yaw)
        print('delta %f, yaw : %f,  gps yaw %f  ' %( m.degrees(delta), m.degrees(yaw), m.degrees(self.gps_yaw)))
        # curve에서는 보정이 안 되고 있는 것 같은데 방안 생각해보기!
        # if abs(delta) > m.radians(2) and abs(delta) < m.radians(90) and self.v > 0.30 and self.path_shape == "straight":  #self.path_shape은 path_opener에서 내보내주기 때문에 path_opener먼저 키기
        if abs(delta) > m.radians(2) and abs(delta) < m.radians(90) and self.v > 0.50 and self.cov < 0.0004:
            if self.decision_straight():
                self.delta = self.mean - yaw
        qx, qy, qz, qw = quaternion_from_euler(0,0,self.gps_yaw)
        dat = Quaternion(x=qx,y=qy,z=qz,w=qw)
        self.pub_gps.publish(dat)
        yaw_prev = yaw
        yaw = yaw + self.delta_yaw + self.delta
        x, y, z, w = quaternion_from_euler(0, 0, yaw)
        print(
            "%.4f   %.4f  %.4f"
            % (m.degrees(yaw_prev), m.degrees(yaw), m.degrees(self.delta))
        )
        data = msg
        data.orientation.x = x
        data.orientation.y = y
        data.orientation.z = z
        data.orientation.w = w
        # scale = 10 ** m.degrees(delta)
        data.orientation_covariance = [
            0.00025000000000000005,
            0.0,
            0.0,
            0.0,
            0.00025000000000000005,
            0.0,
            0.0,
            0.0,
            0.00025000000000000005,
        ]

        self.pub.publish(data)


def main(args=None):
    rclpy.init(args=args)
    node = Rotate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
