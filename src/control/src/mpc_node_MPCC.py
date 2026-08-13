#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import sys
import math
import threading
import numpy as np
import yaml
import csv
import rospkg
from collections import deque
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from morai_msgs.msg import CtrlCmd, TrackingInfo, MPCWeight
from tf.transformations import euler_from_quaternion

from scipy.ndimage import gaussian_filter1d
import hpipm_python as hpipm

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

from test_stanley import stanley, state


class MPCCController:
    def __init__(self):
        rospy.init_node('mpc_trajectory_follower_mpcc', anonymous=True)

        package_path = rospkg.RosPack().get_path('control')
        yaml_path = os.path.join(package_path, 'parameter', 'config.yaml')
        with open(yaml_path, 'r') as f:
            self.__dict__.update(yaml.safe_load(f))

        self.state_lock = threading.Lock()
        self.error_log = deque(maxlen=int(getattr(self, 'error_log_maxlen', 36000)))

        # MPCC 상태 변수 확장: [e_c(Contouring), e_l(Lag), e_yaw, v] -> nx=4
        self.nx = 4
        self.nu = 2  # [accel, steer]

        self.stanley_solver = stanley()
        self.stanley_state = state()
        self.stanley_solver.L = self.wheelbase
        self.stanley_solver.kv = self.stanley_kv

        self.odom_received = False
        self.cx, self.cy, self.cyaw = [], [], []
        self.current_x = self.current_y = self.current_v = self.current_yaw = 0.0
        self.target_v = 11.5
        self.closest_idx = 0
        self.curve_gain = 0.0
        self.ckappa = np.array([], dtype=float)
        self.curve_preview = int(getattr(self, 'future', self.T))
        self.curvature_sigma = float(getattr(self, 'curvature_sigma', 1.0))

        self.v_error_integral = 0.0
        self.prev_accel_cmd = self.last_accel_cmd = self.last_steer_cmd = 0.0
        self.max_accel_rate = float(getattr(self, 'max_accel', 2.0))
        self.max_steer_rate = float(getattr(self, 'max_dsteer', math.radians(60.0)))
        self.mpc_log_period = float(getattr(self, 'mpc_log_period', 0.1))

        # MPCC 가중치 설정
        self.Q_mpcc = np.diag([self.q_ct_base, self.q_v_base, self.q_yaw_base, self.q_v_base * 0.5])
        self.R_mpcc = np.diag([self.r_accel_base, self.r_steer_base])

        self.lbu = np.array([-self.max_accel, -self.max_steer])
        self.ubu = np.array([self.max_accel, self.max_steer])
        self.lbx = np.array([-np.inf, -np.inf, -np.inf, self.min_speed])
        self.ubx = np.array([np.inf, np.inf, np.inf, self.max_speed])

        self.path_csv_path = rospy.get_param('~path_csv_path', '/home/acca/acca_ws/global_path/global_path.csv')
        self.load_path_from_csv(self.path_csv_path)

        self.prev_accel = np.zeros(self.T, dtype=float)
        self.prev_delta = np.zeros(self.T, dtype=float)

        rospy.Subscriber('/localization/kinematic_state', Odometry, self.odom_callback)
        rospy.Subscriber('/global_path', Path, self.path_callback)
        rospy.Subscriber('target_velocity', Float32, self.target_vel_callback)
        rospy.Subscriber('/mpc_weight', MPCWeight, self.weight_callback)

        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)
        self.track_info_pub = rospy.Publisher('/tracking_info', TrackingInfo, queue_size=10)

        self.rate = rospy.Rate(1.0 / self.dt)
        rospy.loginfo("MPCC(윤곽 제어) 노드가 성공적으로 시작되었습니다.")

    def _get_qp_workspace(self):
        if getattr(self, '_qp_ws', None) is None:
            dim = hpipm.hpipm_ocp_qp_dim(self.T)
            dim.set('nx', self.nx, 0, self.T)
            dim.set('nu', self.nu, 0, self.T-1)
            dim.set('nbx', self.nx, 0)
            dim.set('nbu', self.nu, 0, self.T-1)

            qp = hpipm.hpipm_ocp_qp(dim)
            qp_sol = hpipm.hpipm_ocp_qp_sol(dim)
            arg = hpipm.hpipm_ocp_qp_solver_arg(dim, 'speed')
            solver = hpipm.hpipm_ocp_qp_solver(dim, arg)
            self._qp_ws = (dim, qp, qp_sol, arg, solver)
        return self._qp_ws

    def get_mpcc_state_space_matrices(self, v, e_yaw, delta, kappa):
        """MPCC Contouring / Lag 오차 변환 야코비안을 반영한 선형 상태 행렬"""
        v = max(0.1, float(v))
        e_yaw, delta, kappa = float(e_yaw), float(np.clip(delta, -self.max_steer, self.max_steer)), float(kappa)

        A = np.eye(self.nx)
        B = np.zeros((self.nx, self.nu))
        b = np.zeros(self.nx)

        sin_yaw, cos_yaw = math.sin(e_yaw), math.cos(e_yaw)
        sec2_delta = 1.0 / max(math.cos(delta)**2, 1e-8)

        # e_c(k+1) = e_c + v * sin(e_yaw) * dt (Contouring Error)
        A[0, 2] = v * cos_yaw * self.dt
        A[0, 3] = sin_yaw * self.dt
        # e_l(k+1) = e_l + (v * cos(e_yaw) - v_target) * dt (Lag Error)
        A[1, 2] = -v * sin_yaw * self.dt
        A[1, 3] = cos_yaw * self.dt
        # e_yaw(k+1)
        A[2, 3] = (math.tan(delta) / self.wheelbase - kappa) * self.dt
        B[2, 1] = (v / self.wheelbase * sec2_delta) * self.dt
        # v(k+1)
        B[3, 0] = self.dt

        b[0] = -v * e_yaw * cos_yaw * self.dt
        b[1] = (v * e_yaw * sin_yaw - self.target_v) * self.dt
        b[2] = -v * delta / self.wheelbase * sec2_delta * self.dt

        return A, B, b

    def odom_callback(self, msg):
        current_x, current_y = msg.pose.pose.position.x, msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        offset_x = current_x + self.wheelbase / 2 * math.cos(yaw)
        offset_y = current_y + self.wheelbase / 2 * math.sin(yaw)
        v = msg.twist.twist.linear.x

        with self.state_lock:
            self.current_yaw, self.current_x, self.current_y, self.current_v = yaw, offset_x, offset_y, v
        self.odom_received = True

    def path_callback(self, msg):
        self.global_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.cx, self.cy = np.array([pt[0] for pt in self.global_path]), np.array([pt[1] for pt in self.global_path])
        self.rebuild_path_yaw()
        self.ckappa = gaussian_filter1d(self.calc_curvature(self.cx, self.cy), sigma=self.curvature_sigma)
        self.update_curve_gain()

    def update_curve_gain(self):
        if self.ckappa is None or len(self.ckappa) == 0:
            self.curve_gain = 0.0
            return
        end_idx = min(self.closest_idx + self.curve_preview, len(self.ckappa))
        future = np.abs(self.ckappa[self.closest_idx : end_idx])
        if len(future) == 0:
            self.curve_gain = 0.0
            return
        weights = np.exp(-0.3 * np.arange(len(future)))
        self.curve_gain = np.sum(future * weights) / np.sum(weights)

    def weight_callback(self, msg):
        with self.state_lock:
            self.Q_mpcc[0,0], self.Q_mpcc[1,1], self.Q_mpcc[2,2] = msg.q_ct, msg.q_v, msg.q_yaw
            self.R_mpcc[0,0], self.R_mpcc[1,1] = msg.r_accel, msg.r_steer

    def load_path_from_csv(self, csv_path):
        if not csv_path or not os.path.exists(csv_path): return
        try:
            xs, ys, yaws = [], [], []
            with open(csv_path, 'r') as f:
                for row in csv.DictReader(f):
                    xs.append(float(row['x'])); ys.append(float(row['y']))
                    if 'yaw' in row and row['yaw'] != '': yaws.append(float(row['yaw']))
            self.cx, self.cy = np.array(xs), np.array(ys)
            self.global_path = list(zip(self.cx, self.cy))
            if len(yaws) == len(xs): self.cyaw = yaws
            else: self.rebuild_path_yaw()
            self.ckappa = gaussian_filter1d(self.calc_curvature(self.cx, self.cy), sigma=self.curvature_sigma)
            self.update_curve_gain()
        except Exception as e: rospy.logerr(f"CSV 로드 실패: {e}")

    def rebuild_path_yaw(self):
        self.cyaw = [math.atan2(self.cy[i+1]-self.cy[i], self.cx[i+1]-self.cx[i]) for i in range(len(self.cx)-1)]
        if len(self.cyaw) > 0: self.cyaw.append(self.cyaw[-1])

    def target_vel_callback(self, msg): self.target_v = msg.data

    def find_closest_waypoint(self, x, y):
        if len(self.cx) < 2: return None, None, None
        search_range = 50
        start_idx = max(0, self.closest_idx - search_range)
        end_idx = min(len(self.cx), self.closest_idx + search_range)
        min_dist, best_idx = float('inf'), self.closest_idx
        for i in range(start_idx, end_idx):
            dist = math.sqrt((x - self.cx[i])**2 + (y - self.cy[i])**2)
            if dist < min_dist: min_dist, best_idx = dist, i
        if min_dist > 5.0:
            for i in range(len(self.cx)):
                dist = math.sqrt((x - self.cx[i])**2 + (y - self.cy[i])**2)
                if dist < min_dist: min_dist, best_idx = dist, i
        self.closest_idx = best_idx
        return self.cx[best_idx], self.cy[best_idx], self.cyaw[best_idx]

    def calc_curvature(self, x, y):
        dx, dy = np.gradient(x), np.gradient(y)
        ddx, ddy = np.gradient(dx), np.gradient(dy)
        return (dx * ddy - dy * ddx) / np.maximum((dx**2 + dy**2)**1.5, 1e-8)

    def run_stanley_imported_fallback(self):
        s = self.stanley_state
        for name, val in [('x', self.current_x), ('y', self.current_y), ('yaw', self.current_yaw), ('v', self.current_v)]:
            if hasattr(s, name): setattr(s, name, val)
        steer = self.stanley_solver.stan_control(self.stanley_state, self.cx, self.cy, self.cyaw, self.stanley_h_gain, self.stanley_c_gain)
        v_error = (self.target_v * 0.7) - self.current_v
        self.v_error_integral = np.clip(self.v_error_integral + v_error * self.dt, -2.0, 2.0)
        accel_cmd = np.clip((self.kp * v_error) + (self.ki * self.v_error_integral), self.prev_accel_cmd - 2.0*self.dt, self.prev_accel_cmd + 2.0*self.dt)
        self.prev_accel_cmd = accel_cmd
        return steer, max(0.0, accel_cmd), max(0.0, -accel_cmd)

    def run(self):
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            with self.state_lock:
                cur_x, cur_y, cur_yaw, cur_v = self.current_x, self.current_y, self.current_yaw, self.current_v
                Q_now, R_now = self.Q_mpcc, self.R_mpcc

            target_x, target_y, target_yaw = self.find_closest_waypoint(cur_x, cur_y)
            if target_x is None:
                self.rate.sleep()
                continue

            self.update_curve_gain()
            e_yaw = math.atan2(math.sin(cur_yaw - target_yaw), math.cos(cur_yaw - target_yaw))
            dx, dy = cur_x - target_x, cur_y - target_y
            
            # Contouring Error (e_c) & Lag Error (e_l) 분리
            e_c = -dx * math.sin(target_yaw) + dy * math.cos(target_yaw)
            e_l = -dx * math.cos(target_yaw) - dy * math.sin(target_yaw)
            self.error_log.append([rospy.get_time() - start_time, e_c, cur_v])

            x0 = np.array([e_c, e_l, e_yaw, cur_v])
            dim, qp, qp_sol, arg, solver = self._get_qp_workspace()

            # QP 행렬 준비
            for t in range(self.T):
                idx = min(self.closest_idx + int(t * cur_v * self.dt / 0.1), len(self.ckappa)-1)
                A_d, B_d, b_d = self.get_mpcc_state_space_matrices(cur_v, e_yaw, self.prev_delta[t], self.ckappa[idx])
                qp.set('A', A_d, t); qp.set('B', B_d, t); qp.set('b', b_d, t)
                qp.set('Q', Q_now, t); qp.set('R', R_now, t)
                qp.set('q', np.zeros(self.nx), t); qp.set('r', np.zeros(self.nu), t)
                qp.set('Jbu', np.eye(self.nu), t); qp.set('lbu', self.lbu, t); qp.set('ubu', self.ubu, t)
                if t > 0:
                    qp.set('Jbx', np.eye(self.nx), t); qp.set('lbx', self.lbx, t); qp.set('ubx', self.ubx, t)

            qp.set('Q', Q_now, self.T); qp.set('q', np.zeros(self.nx), self.T)
            qp.set('Jbx', np.eye(self.nx), self.T); qp.set('lbx', self.lbx, self.T); qp.set('ubx', self.ubx, self.T)
            qp.set('Jbx', np.eye(self.nx), 0); qp.set('lbx', x0, 0); qp.set('ubx', x0, 0)

            solver.solve(qp, qp_sol)
            if int(solver.get('status')) == 0:
                u0 = np.asarray(qp_sol.get('u', 0), dtype=float).reshape(-1)
                accel_cmd, steer_cmd = float(u0[0]), float(u0[1])
                self.prev_delta[:-1] = [qp_sol.get('u', i)[1] for i in range(1, self.T)]
                self.prev_delta[-1] = self.prev_delta[-2]
                if accel_cmd >= 0.0: accel, brake = accel_cmd, 0.0
                else:
                    accel = 0.0
                    brake = 0.0 if accel_cmd > -0.3 else min(-accel_cmd - 0.3, getattr(self, 'max_brake', self.max_accel))
            else:
                steer_cmd, accel, brake = self.run_stanley_imported_fallback()

            track_msg = TrackingInfo(cross_track_error=e_c, heading_error=e_yaw, curvature_gain=self.curve_gain, steering=steer_cmd)
            self.track_info_pub.publish(track_msg)

            ctrl_msg = CtrlCmd(ctrl_mode=2, cmd_type=1, gear=4, steer=steer_cmd,
                               accel=np.clip(accel/self.max_accel, 0.0, 1.0), brake=np.clip(brake/getattr(self, 'max_brake', self.max_accel), 0.0, 1.0))
            self.ctrl_pub.publish(ctrl_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        MPCCController().run()
    except rospy.ROSInterruptException:
        pass