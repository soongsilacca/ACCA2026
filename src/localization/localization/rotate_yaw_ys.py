#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, Quaternion, PoseWithCovarianceStamped
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
        self.create_subscription(PoseWithCovarianceStamped, '/pcl_pose', self.callback_pclpose, qos_profile)
        self.create_subscription(String, "jamming_status", self.callback_jamming, qos_profile)

        # self.create_subscription(String, "road_type", self.callback_shape, qos_profile)
        # self.create_subscription(Odometry, "odometry/navsat",self.callback_odom, qos_profile)

        self.pub = self.create_publisher(Imu, "imu/rotated", qos_profile)

        self.pub_gps = self.create_publisher(Quaternion, "mean", qos_profile)

        self.log_counter = 0   # 로그 카운터 변수


        self.delta_yaw = self.declare_parameter("delta_yaw", 0.0).value
        self.gps_yaw = 0.0
        self.ndt_yaw = 0.0  
        self.v = 0.0
        self.odom_yaw = 0.0
        self.delta = 0.0
        self.path_shape = "straight"
        self.gps_forward = [
            None
        ] * 10  # 큐 사이즈 10 고려해보기 -> 아래 decision_straight함수에서 하나씩 꺼내서 다 검사하기 때문에 연산 시간 때문에 조금 줄여야 할 수도 있음 / 심지어 imu는 400hz
        self.ndt_forward = [None] * 10
        self.mean = 0.0
        self.cov = 0.0
        self.mode = 'False'  # noraml_state = False / Tunnel = True

    def decision_straight(self, forward):
        sum = 0.0
        for f in forward:
            if f == None:
                return False
            else:
                sum += f
        mean = sum / len(forward)
        for f in forward:
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
        self.gps_yaw = m.atan2(vy, vx)
        self.gps_forward.append(self.gps_yaw)
        del self.gps_forward[0]
    
    def callback_jamming(self,msg):
        if msg.data == "False":
            self.mode = 'False'

        elif msg.data == "True":
            self.mode = 'True'

    def callback_pclpose(self,msg):
        msg = msg.pose.pose
        _,_,ndt_yaw = euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
        self.ndt_forward.append(ndt_yaw)
        self.ndt_yaw = ndt_yaw
        del self.ndt_forward[0]


    # def callback_odom(self,msg):
    #     _,_,self.odom_yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

    # def callback_shape(self, msg):
    #     self.path_shape = msg.data

    def callback(self, msg):
        _, _, yaw = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )
        self.raw_yaw = yaw
        self.delta_gps = normalize_angle(self.gps_yaw - yaw)
        self.delta_ndt = normalize_angle(self.ndt_yaw - yaw)
        self.update_delta()
        # print('delta %f, yaw : %f,  gps yaw %f  ' %( m.degrees(self.delta), m.degrees(yaw), m.degrees(self.gps_yaw)))
        # curve에서는 보정이 안 되고 있는 것 같은데 방안 생각해보기!
        # if abs(delta) > m.radians(2) and abs(delta) < m.radians(90) and self.v > 0.30 and self.path_shape == "straight":  #self.path_shape은 path_opener에서 내보내주기 때문에 path_opener먼저 키기

        qx, qy, qz, qw = quaternion_from_euler(0,0,self.gps_yaw)
        dat = Quaternion(x=qx,y=qy,z=qz,w=qw)
        self.pub_gps.publish(dat)
        yaw_prev = yaw
        yaw = yaw + self.delta_yaw + self.delta
        x, y, z, w = quaternion_from_euler(0, 0, yaw)
        if self.log_counter % 10 == 0:
            print(
                "%.4f   %.4f  %.4f %.4f"
                % (m.degrees(yaw_prev), m.degrees(yaw), m.degrees(self.delta), m.degrees(self.ndt_yaw))
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

    def update_delta(self):
        self.log_counter += 1
        if self.mode == 'False':
            if abs(self.delta_gps) > m.radians(2) and abs(self.delta_gps) < m.radians(90) and self.v > 0.50 and self.cov < 0.0004:
                if self.decision_straight(self.gps_forward):
                    self.delta = self.mean - self.raw_yaw
        elif self.mode == 'True':
            if self.log_counter % 10 == 0:
                self.get_logger().info('using ndt')

            if abs(self.delta_ndt) > m.radians(2) and abs(self.delta_ndt) < m.radians(90):
                if self.decision_straight(self.ndt_forward):
                    self.delta = self.mean - self.raw_yaw
                    self.get_logger().info('rotated by ndt')



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
