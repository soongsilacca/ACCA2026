#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, Int32, String, Bool
from erp42_msgs.msg import SerialFeedBack, ControlMessage
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray
from stanley_cone import Stanley
from .mpc_node_cone import MPC
from scipy.ndimage import gaussian_filter1d
# from tf_transformations import *
import numpy as np
import math as m
import threading
# from pyproj import *
from random import randint
import os
import sqlite3
from DB import DB
import time
from scipy.interpolate import CubicSpline, UnivariateSpline
from scipy import interpolate

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

class PID:
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain", 0.85).value
        # self.p_gain = node.declare_parameter("/stanley_controller/p_gain", 1.0).value
        # self.i_gain = node.declare_parameter("/stanley_controller/i_gain", 0.05).value

        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0

        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )

    def PIDControl(self, speed, desired_value, min, max):

        self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (
            self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        dt = self.current - self.last
        self.last = self.current

        err = desired_value - speed
        # self.d_err = (err - self.p_err) / dt

        self.p_err = err
        self.i_err += self.p_err * dt * (0.0 if speed == 0 else 1.0)
        if self.i_err > 5.0:
            self.i_err = 5.0
        if self.i_err < -5.0:
            self.i_err = -5.0

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)

        return int(np.clip(self.speed, min, max))

class PathHandler():
    def __init__(self, node, path_topic, one_topic):
        self.node = node
        self.node.create_subscription(Path, path_topic, self.callback_path, qos_profile_system_default)
        self.node.create_subscription(Bool, one_topic, self.done_path, qos_profile_system_default)
        

        self.cx = []
        self.cy = []
        self.cyaw = []

        self.db_path = os.path.expanduser("/home/acca/db_file/mpc/path_data.db")
        self.init_db()
        self.path = False
        self.one = False

    def init_db(self):
        thread_id = threading.get_ident()
        self.node.get_logger().info(f'[init_db] Thread ID : {thread_id}')
        with sqlite3.connect(self.db_path) as conn:
            cur = conn.cursor()
            cur.execute('''
                CREATE TABLE IF NOT EXISTS path (
                    path_id  CHAR(4),
                    idx INTEGER PRIMARY KEY AUTOINCREMENT,
                    x REAL, y REAL, yaw REAL, speed REAL
                )
            ''')

    def compute_curvature(self, cx, cy):
        kappa = []
        for i in range(1, len(cx) - 1):
            x1, y1 = cx[i - 1], cy[i - 1]
            x2, y2 = cx[i], cy[i]
            x3, y3 = cx[i + 1], cy[i + 1]

            dx1 = x2 - x1
            dy1 = y2 - y1
            dx2 = x3 - x2
            dy2 = y3 - y2

            angle1 = m.atan2(dy1, dx1)
            angle2 = m.atan2(dy2, dx2)

            dtheta = angle2 - angle1
            dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi

            ds = m.hypot(dx1, dy1)
            if ds == 0:
                kappa.append(0.0)
            else:
                kappa.append(abs(dtheta) / ds)

        # 양 끝 보정
        if len(kappa) > 0:
            kappa = [kappa[0]] + kappa + [kappa[-1]]
        else:
            kappa = [0.0] * len(cx)

        return np.array(kappa)

    def kappa_to_speed(self, kappa):
        max_speed = 15.0  # km/h
        min_speed = 8.0
        kappa = np.clip(kappa, 0.0, 0.5)
        alpha = 5.0
        speed = min_speed + (max_speed - min_speed) * np.exp(-alpha * kappa)
        speed = np.clip(speed, min_speed, max_speed)
        # speed = max_speed - (kappa / 0.5) * (max_speed - min_speed)
        
        return speed
    
    def smooth_speed(self, speeds, max_delta=0.5):
        # speeds : numpy array
        smoothed = speeds.copy()
        for i in range(1, len(speeds)):
            delta = smoothed[i] - smoothed[i-1]
            if abs(delta) > max_delta:
                smoothed[i] = smoothed[i-1] + np.sign(delta) * max_delta
        return smoothed
    
    def resample_path(self, cx, cy, spacing=0.005):
        """균일한 간격(spacing)으로 (x, y) 경로 리샘플링"""
        

        # 누적 거리 계산
        dx = np.diff(cx)
        dy = np.diff(cy)
        dist = np.sqrt(dx**2 + dy**2)
        cumulative = np.insert(np.cumsum(dist), 0, 0)

        total_length = cumulative[-1]
        if total_length < spacing:
            return cx, cy  # 너무 짧은 경로는 리샘플링하지 않음

        n_samples = int(total_length / spacing)
        uniform_dist = np.linspace(0, total_length, n_samples)

        fx = interpolate.interp1d(cumulative, cx, kind='linear')
        fy = interpolate.interp1d(cumulative, cy, kind='linear')

        rx = fx(uniform_dist)
        ry = fy(uniform_dist)

        return rx.tolist(), ry.tolist()

    def done_path(self, msg):
        self.one = msg.data
        thread_id = threading.get_ident()
        self.node.get_logger().info(f"[done_path] Thread ID: {thread_id}")
        if msg.data:
            try:
                str_time = time.time()               
                 
                with sqlite3.connect(self.db_path) as conn:

                    self.node.get_logger().info(f"[done_path] Opened DB connection in thread ID: {thread_id}")
                    cur = conn.cursor()

                    # ✅ 중복 제거
                    unique_coords = set()
                    filtered_x, filtered_y = [], []

                    for x_val, y_val in zip(self.cx, self.cy):
                        key = (round(x_val, 1), round(y_val, 1))
                        if key not in unique_coords:
                            unique_coords.add(key)
                            filtered_x.append(x_val)
                            filtered_y.append(y_val)

                    raw_cx, raw_cy = filtered_x, filtered_y

                    # ✅ 리샘플링 적용
                    self.cx, self.cy = self.resample_path(raw_cx, raw_cy, spacing=0.1)

                    raw_kappa = self.compute_curvature(self.cx, self.cy)
                    # smoothed_kappa = np.convolve(raw_kappa, np.ones(5) / 5, mode="same")  # smoothing window = 5
                    smoothed_kappa = gaussian_filter1d(raw_kappa, sigma=2)
                    speeds = self.kappa_to_speed(smoothed_kappa)
                    speeds = self.smooth_speed(speeds, max_delta=0.3)  # 최대 0.3 km/h 변화 제한

                    # 기존 경로
                    x = np.array(self.cx)
                    y = np.array(self.cy)
                    yaw_list = []
                    for row in self.way.poses:
                        q = row.pose.orientation
                        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                        yaw_list.append(yaw)
                    # 더 부드러운 보간을 위해: 끝에서 5개, 시작에서 5개를 추출하여 이어줌
                    n_edge = 3  # 사용할 점 개수
                    x_edge = np.concatenate([x[-n_edge:], x[:n_edge]])
                    y_edge = np.concatenate([y[-n_edge:], y[:n_edge]])
                    yaw_edge = np.concatenate([yaw_list[-n_edge:], yaw_list[:n_edge]])

                    # 누적 거리로 정규화된 t_edge 계산
                    distances = [0.0]
                    for i in range(1, len(x_edge)):
                        dx = x_edge[i] - x_edge[i - 1]
                        dy = y_edge[i] - y_edge[i - 1]
                        distances.append(distances[-1] + np.hypot(dx, dy))

                    t_edge = np.array(distances) / distances[-1]

                    # 인덱스를 0 ~ 1로 정규화
                    # t_edge = np.linspace(0, 1, len(x_edge))

                    # 보간 함수
                    cs_x = CubicSpline(t_edge, x_edge, bc_type='natural')
                    cs_y = CubicSpline(t_edge, y_edge, bc_type='natural')
                    cs_yaw = CubicSpline(t_edge, yaw_edge, bc_type='natural')

                    # 연결 구간 보간 (예: 10개)
                    t_insert = np.linspace(0, 1, 30, endpoint=False)
                    extra_x = cs_x(t_insert)
                    extra_y = cs_y(t_insert)
                    extra_yaw = cs_yaw(t_insert)

                    extra_raw_kappa = self.compute_curvature(extra_x, extra_y)
                    # smoothed_kappa = np.convolve(raw_kappa, np.ones(5) / 5, mode="same")  # smoothing window = 5
                    extra_smoothed_kappa = gaussian_filter1d(extra_raw_kappa, sigma=2)
                    extra_speeds = self.kappa_to_speed(extra_smoothed_kappa)
                    extra_speeds = self.smooth_speed(extra_speeds, max_delta=0.3)  # 최대 0.3 km/h 변화 제한
                    
                    # smoothing factor s를 0으로 하면 점들을 모두 통과하는 스플라인 생성
                    spline_x = UnivariateSpline(t_insert, extra_x, s=2, k=3)
                    spline_y = UnivariateSpline(t_insert, extra_y, s=2, k=3)

                    spline_yaw = UnivariateSpline(t_insert, extra_yaw, s=3, k=3)

                    # spline_x(t), spline_y(t)로 t 범위 내에서 부드러운 x,y 좌표를 얻을 수 있음
                    # 예를 들어 100개의 부드러운 점 생성
                    t_smooth = np.linspace(t_insert.min(), t_insert.max(), len(extra_speeds))
                    smooth_x = spline_x(t_smooth)
                    smooth_y = spline_y(t_smooth)
                    smooth_yaw = spline_yaw(t_smooth)

                    # 기존 경로 저장
                    min_len = min(len(self.cx), len(self.cy), len(self.way.poses), len(speeds))
                    for i in range(min_len):
                        x = self.cx[i]
                        y = self.cy[i]
                        q = self.way.poses[i].pose.orientation
                        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                        speed = int(speeds[i])

                        cur.execute(
                            "INSERT INTO Path (path_id, x, y, yaw, speed) VALUES (?, ?, ?, ?, ?)",
                            ("A1A2", x, y, yaw, speed)
                        )

                    # 시작점과 끝점 사이 부드럽게 이어주는 구간 추가 삽입
                    for i in range(len(extra_x)):
                        cur.execute(
                            "INSERT INTO Path (path_id, x, y, yaw, speed) VALUES (?, ?, ?, ?, ?)",
                            ("A1A2", float(smooth_x[i]), float(smooth_y[i]), float(smooth_yaw[i]), int(extra_speeds[i]))
                        )                        

                    conn.commit()
                    end_time = time.time()
                    print("latency", end_time-str_time)
                    self.node.get_logger().info("[done_path] Path with curvature-based speed saved to DB.")
                    self.path = True

            except Exception as e:
                self.node.get_logger().warn(f"[done_path] DB error: {e}")

    def callback_path(self, msg):
        self.way = msg
        self.cx, self.cy, self.cyaw = self.update_path(msg)

    def update_path(self, data):
        cx = []
        cy = []
        cyaw = []
        for p in data.poses:
            cx.append(p.pose.position.x)
            cy.append(p.pose.position.y)
            _, _, yaw = euler_from_quaternion([
                p.pose.orientation.x,
                p.pose.orientation.y,
                p.pose.orientation.z,
                p.pose.orientation.w
            ])
            cyaw.append(yaw)
        return cx, cy, cyaw

class State():
    def __init__(self, node, odom_topic):
        node.create_subscription(Odometry, odom_topic, self.callback, qos_profile_system_default)
        node.create_subscription(SerialFeedBack, "erp42_feedback", self.callback_erp, qos_profile_system_default)

        self.x = 0.  # m
        self.y = 0.  # m
        self.yaw = 0.  # rad
        self.v = 0.  # m/s

    def callback(self,msg):
        self.pose = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _,_,self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])    
    
    def callback_erp(self,msg):
        self.speed = msg
        self.v = msg.speed
        # print(f"[ERP Callback] Received speed: {self.v}")
    
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

class SpeedSupporter():
    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self,value,hdr,ctr,min_value,max_value, he_gain, ce_gain, he_thr, ce_thr):
        hdr = self.func(abs(hdr), -he_gain, he_thr)
        ctr = self.func(abs(ctr), -ce_gain, ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)
        return res
                      


class Drive():
    def __init__(self, node, state, path):
        self.pub = node.create_publisher(ControlMessage, "cmd_msg", qos_profile_system_default)
        node.create_subscription(Float32, "hdr", self.hdr_callback,qos_profile_system_default)
        node.create_subscription(Float32, "ctr", self.ctr_callback, qos_profile_system_default)

        self.hdr = 0.0
        self.ctr = 0.0
        
        self.path = path
        self.state = state
 
        self.st = Stanley()
        self.ss = SpeedSupporter()
        self.pid = PID(node)
        self.mpc = None

        self.first_lap = True


    def set_mpc(self, mpc_instance):
        self.mpc = mpc_instance

    def hdr_callback(self, msg):
        self.hdr = msg.data

    def ctr_callback(self, msg):
        self.ctr = msg.data

    def publish_cmd(self):        
        target_idx, error  = self.st.calc_target_index(self.state, self.path.cx, self.path.cy)
        self.decision_first_lap(target_idx)
        

        if not self.path.one: # first lap
            # print(self.path.one)
            if self.decision_last_idx(target_idx):
                h_gain_curve = 0.8
                c_gain_curve = 0.5
                target_speed = 1.0

                steer, hdr, ctr = self.st.stanley_control(self.state, self.path.cyaw, h_gain_curve, c_gain_curve, target_idx, error)
                adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=2, max_value=4, he_gain=50.0, ce_gain=30.0, he_thr=0.001, ce_thr=0.002)
                if self.state.v * 3.6 >= adapted_speed:
                    input_brake = (abs(self.state.v * 3.6 - adapted_speed) / 20.0) * 200
                else:
                    input_brake = 0

                print("close last idx", adapted_speed)
                speed = adapted_speed

            else:
                
                if self.decision_straight(target_idx):
                    # h_gain_straight = 0.5
                    # c_gain_straight = 0.24

                    h_gain_straight = 0.6
                    c_gain_straight = 0.3
                    target_speed = 10.0
                    # target_speed = 5.0

                    steer, hdr, ctr = self.st.stanley_control(self.state, self.path.cyaw, h_gain_straight, c_gain_straight, target_idx, error)
                    adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=8, max_value=15, he_gain=40.0, ce_gain=30.0, he_thr=0.07, ce_thr=0.05)
                    if self.state.v * 3.6 >= adapted_speed:
                        input_brake = (abs(self.state.v * 3.6 - adapted_speed) / 20.0) * 200
                    else:
                        input_brake = 0
                    speed = adapted_speed

                    print("straight", adapted_speed)


                else:
                    h_gain_curve = 0.8
                    c_gain_curve = 0.5
                    target_speed = 7.0

                    steer, hdr, ctr = self.st.stanley_control(self.state, self.path.cyaw, h_gain_curve, c_gain_curve, target_idx, error)
                    adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=6, max_value=7, he_gain=50.0, ce_gain=30.0, he_thr=0.001, ce_thr=0.002)
                    if self.state.v * 3.6 >= adapted_speed:
                        input_brake = (abs(self.state.v * 3.6 - adapted_speed) / 20.0) * 200
                    else:
                        input_brake = 0
                    speed = adapted_speed


                    print("curve", adapted_speed)


        else: # not first lap
            # print(self.path.one)
            if self.decision_straight(target_idx):                
                # h_gain_straight = 0.6
                # c_gain_straight = 0.4
                # target_speed = 15.0
#######################################################################################################################################################
                # steer, hdr, ctr = self.st.stanley_control(self.state, self.path.cyaw, h_gain_straight, c_gain_straight, target_idx, error)
                _, steer, speed_output = self.mpc.pose_callback(self.state.pose, self.state.v )
                # adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=13, max_value=15, he_gain=40.0, ce_gain=20.0, he_thr=0.03, ce_thr=0.05)
                kspeed = speed_output * 3.6
                # adapted_speed = -50.0
                if self.hdr != 0 and self.ctr != 0:
                    adapted_speed = self.ss.adaptSpeed(
                        kspeed, self.hdr, self.ctr, min_value=5, max_value=15
                    )
                    if self.state.v * 3.6 >= adapted_speed:
                        input_brake = (abs(self.state.v * 3.6 - adapted_speed) / 20.0) * 200
                    else:
                        input_brake = 0
                    speed = adapted_speed
                else:
                    if self.state.v * 3.6 >= kspeed:
                        input_brake = (abs(self.state.v * 3.6 - kspeed) / 20.0) * 200
                    else:
                        input_brake = 0
                    speed = kspeed

                speed = self.pid.PIDControl(self.state.v * 3.6, speed, 0, 25)
                # if adapted_speed == -50.0:
                #     speed = self.pid.PIDControl(
                #         self.odometry.v * 3.6, kspeed, 0, 25
                #     )  # speed 조정 (PI control)
                # else:
                #     speed = self.pid.PIDControl(
                #         self.odometry.v * 3.6, adapted_speed, 0, 25
                #     )   
                # print("straight", adapted_speed)

            else:
                # h_gain_curve = 0.8
                # c_gain_curve = 0.5
                # target_speed = 8

                # steer, hdr, ctr = self.st.stanley_control(self.state, self.path.cyaw, h_gain_curve, c_gain_curve, target_idx, error)
                # adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=5, max_value=8, he_gain=50.0, ce_gain=30.0, he_thr=0.001, ce_thr=0.002)
                _, steer, speed_output = self.mpc.pose_callback(self.state.pose , self.state.v)
                kspeed = speed_output * 3.6
                # adapted_speed = -50.0
                if self.hdr != 0 and self.ctr != 0:
                    adapted_speed = self.ss.adaptSpeed(
                        kspeed, self.hdr, self.ctr, min_value=5, max_value=15
                    )
                    if self.state.v * 3.6 >= adapted_speed:
                        input_brake = (abs(self.state.v * 3.6 - adapted_speed) / 20.0) * 200
                    else:
                        input_brake = 0
                    speed = adapted_speed
                else:
                    if self.state.v * 3.6 >= kspeed:
                        input_brake = (abs(self.state.v * 3.6 - kspeed) / 20.0) * 200
                    else:
                        input_brake = 0
                    speed = kspeed
                speed = self.pid.PIDControl(self.state.v * 3.6, speed, 0, 25)

                # print("curve", adapted_speed)

        msg = ControlMessage()
        # msg.speed = max(0, min(65535, int(round(speed)) * 10)) 
        msg.speed = int(speed)*10 
        msg.steer = int(m.degrees((-1)*steer))
        msg.gear = 2
        msg.brake = int(input_brake)

        self.pub.publish(msg)

    def decision_first_lap(self, target_idx):
        if len(self.path.cyaw) >= 100 and target_idx <= 10: #생성된 path가 10m 이상이고 target_idx가 10 이하일 떄, 즉 출발점 부근일 떼
            self.first_lap = False

    def decision_last_idx(self, target_idx):
        if  abs(len(self.path.cyaw) - target_idx) <= 10: # path의 마지막 노드랑 차랑 인덱스가 10개 이내일때 (즉 path의 마지막 노드랑 거리가 1m 이내일 때)
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
        # print(mean)
        if mean > 0.018: #1027 0.01 -> 0.0075 -> 0.015 -> 0.02
            return False
        else:
            return True
        

def main(args = None):
    rclpy.init(args = args)
    
    node = rclpy.create_node("driving_node")
    state = State(node, "/odometry/navsat")
    path_tracking = PathHandler(node, "del_path", "one_lap_done")
    d = Drive(node, state, path_tracking)
    
    thread = threading.Thread(target=rclpy.spin, args= (node, ), daemon = True)
    thread.start()

    rate = node.create_rate(8)
    # rate_sleep = node.create_rate(1)

    # first_rap_done = False
    mpc_ready = False
    first = True
    db = None
    db_path = "/mpc/path_data.db"
    # if not mpc_ready and path_tracking.path and first:                        
    #     db_path = "path_data.db"
    #     db = DB(db_path)
       

    while rclpy.ok():
        try:
            
                # rate_sleep.sleep()
            # print("*************log****************")
            # print(mpc_ready)
            # print(path_tracking.path)
            if not mpc_ready and path_tracking.path and first:
                if db is None:  # db가 아직 없다면 생성
                    
                    db = DB(db_path)
                    first = False
                    
            if not mpc_ready and path_tracking.path:
                time.sleep(0.15)
                mpc = MPC(db)
                d.set_mpc(mpc)
                mpc_ready = True 
            # print("*************log****************")
            # if not mpc_ready and path_tracking.path:
            #     mpc = MPC(db)
            #     d.set_mpc(mpc)
                                
            d.publish_cmd()

            # if not d.first_lap and not first_rap_done:
            #     print('First lap complete. Waiting before seocond lap...')
            #     rate.sleep()
            #     rate.sleep()
            #     first_rap_done = True

        except Exception as ex:
            print(ex)
        rate.sleep()
    
    
    
if __name__=="__main__":
    main()