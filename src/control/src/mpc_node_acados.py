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

# acados & CasADi 임포트
try:
    from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
    import casadi as ca
except ImportError:
    rospy.logerr("acados_template 또는 casadi가 설치되지 않았습니다.")

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

from test_stanley import stanley, state


class AcadosMPCController:
    def __init__(self):
        rospy.init_node('mpc_trajectory_follower_acados', anonymous=True)

        package_path = rospkg.RosPack().get_path('control')
        yaml_candidates = [
            os.path.join(package_path, 'config', 'config.yaml'),
            os.path.join(package_path, 'src', 'config.yaml'),
            os.path.join(package_path, 'parameter', 'config.yaml'),
        ]
        yaml_path = next((path for path in yaml_candidates if os.path.exists(path)), None)
        if yaml_path is None:
            raise FileNotFoundError("config.yaml을 찾을 수 없습니다.")

        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        self.__dict__.update(config)

        self.state_lock = threading.Lock()
        self.error_log = deque(maxlen=int(getattr(self, 'error_log_maxlen', 36000)))

        # Stanley Fallback 초기화
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

        self.Q = np.diag([self.q_ct_base, self.q_v_base, self.q_yaw_base])
        self.R = np.diag([self.r_accel_base, self.r_steer_base])

        self.path_csv_path = rospy.get_param('~path_csv_path', '/home/acca/acca_ws/global_path/global_path.csv')
        self.load_path_from_csv(self.path_csv_path)

        # acados NMPC 솔버 생성
        self.ocp_solver = self._setup_acados_solver()

        rospy.Subscriber('/localization/kinematic_state', Odometry, self.odom_callback)
        rospy.Subscriber('/global_path', Path, self.path_callback)
        rospy.Subscriber('target_velocity', Float32, self.target_vel_callback)
        rospy.Subscriber('/mpc_weight', MPCWeight, self.weight_callback)

        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)
        self.track_info_pub = rospy.Publisher('/tracking_info', TrackingInfo, queue_size=10)

        self.rate = rospy.Rate(1.0 / self.dt)
        rospy.loginfo("acados NMPC 제어 노드가 시작되었습니다.")

    def _setup_acados_solver(self):
        """CasADi 심볼릭 모델 및 acados OCP 솔버를 초기화합니다."""
        model = AcadosModel()
        model.name = 'kinematic_bicycle_nmpc'

        # 상태 변수: [e_ct, v, e_yaw]
        e_ct = ca.SX.sym('e_ct')
        v = ca.SX.sym('v')
        e_yaw = ca.SX.sym('e_yaw')
        x = ca.vertcat(e_ct, v, e_yaw)

        # 제어 입력: [accel, steer]
        accel = ca.SX.sym('accel')
        steer = ca.SX.sym('steer')
        u = ca.vertcat(accel, steer)

        # 온라인 파라미터: 곡률 [kappa]
        kappa = ca.SX.sym('kappa')
        p = ca.vertcat(kappa)

        # 비선형 운동학 미분 방정식 (ODE)
        x_dot = ca.SX.sym('x_dot', 3)
        f_expl = ca.vertcat(
            v * ca.sin(e_yaw),
            accel,
            v / self.wheelbase * ca.tan(steer) - v * kappa
        )
        model.f_expl_expr = f_expl
        model.f_impl_expr = x_dot - f_expl
        model.x = x
        model.u = u
        model.xdot = x_dot
        model.p = p

        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = self.T
        ocp.solver_options.tf = self.T * self.dt

        # 비용 함수 설정 (NONLINEAR_LS)
        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.model.cost_y_expr = ca.vertcat(x, u)
        ocp.model.cost_y_expr_e = x

        ocp.cost.W = np.block([
            [self.Q, np.zeros((3, 2))],
            [np.zeros((2, 3)), self.R]
        ])
        ocp.cost.W_e = np.diag([self.qf_ct, self.qf_v, self.qf_yaw])

        ocp.cost.yref = np.zeros(5)
        ocp.cost.yref_e = np.zeros(3)

        # 제약 조건
        ocp.constraints.lbu = np.array([-self.max_accel, -self.max_steer])
        ocp.constraints.ubu = np.array([self.max_accel, self.max_steer])
        ocp.constraints.idxbu = np.array([0, 1])
        ocp.constraints.lbx = np.array([-np.inf, self.min_speed, -np.inf])
        ocp.constraints.ubx = np.array([np.inf, self.max_speed, np.inf])
        ocp.constraints.idxbx = np.array([0, 1, 2])
        ocp.constraints.x0 = np.zeros(3)

        # 파라미터 초기화
        ocp.parameter_values = np.array([0.0])

        # 솔버 옵션 (RTI - Real Time Iteration)
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.qp_solver_cond_N = self.T

        return AcadosOcpSolver(ocp, json_file='acados_ocp.json')

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(quaternion)

        offset_x = current_x + self.wheelbase / 2 * math.cos(yaw)
        offset_y = current_y + self.wheelbase / 2 * math.sin(yaw)
        v = msg.twist.twist.linear.x

        with self.state_lock:
            self.current_yaw = yaw
            self.current_x = offset_x
            self.current_y = offset_y
            self.current_v = v
        self.odom_received = True

    def path_callback(self, msg):
        self.global_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.cx = np.array([pt[0] for pt in self.global_path])
        self.cy = np.array([pt[1] for pt in self.global_path])
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
            self.Q = np.diag([msg.q_ct, msg.q_v, msg.q_yaw])
            self.R = np.diag([msg.r_accel, msg.r_steer])

    def load_path_from_csv(self, csv_path):
        if not csv_path or not os.path.exists(csv_path):
            return
        try:
            xs, ys, yaws = [], [], []
            with open(csv_path, 'r', newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    xs.append(float(row['x']))
                    ys.append(float(row['y']))
                    if 'yaw' in row and row['yaw'] != '':
                        yaws.append(float(row['yaw']))
            self.cx, self.cy = np.array(xs), np.array(ys)
            self.global_path = list(zip(self.cx, self.cy))
            if len(yaws) == len(xs): self.cyaw = yaws
            else: self.rebuild_path_yaw()
            self.ckappa = gaussian_filter1d(self.calc_curvature(self.cx, self.cy), sigma=self.curvature_sigma)
            self.update_curve_gain()
        except Exception as e:
            rospy.logerr(f"CSV 로드 실패: {e}")

    def rebuild_path_yaw(self):
        self.cyaw = []
        for i in range(len(self.cx) - 1):
            self.cyaw.append(math.atan2(self.cy[i+1] - self.cy[i], self.cx[i+1] - self.cx[i]))
        if len(self.cyaw) > 0: self.cyaw.append(self.cyaw[-1])

    def target_vel_callback(self, msg):
        self.target_v = msg.data

    def find_closest_waypoint(self, x, y):
        if len(self.cx) < 2: return None, None, None
        search_range = 50
        start_idx = max(0, self.closest_idx - search_range)
        end_idx = min(len(self.cx), self.closest_idx + search_range)
        min_dist = float('inf')
        best_idx = self.closest_idx
        for i in range(start_idx, end_idx):
            dist = math.sqrt((x - self.cx[i])**2 + (y - self.cy[i])**2)
            if dist < min_dist:
                min_dist = dist
                best_idx = i
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
                Q_now, R_now = self.Q, self.R

            target_x, target_y, target_yaw = self.find_closest_waypoint(cur_x, cur_y)
            if target_x is None:
                self.rate.sleep()
                continue

            self.update_curve_gain()
            e_yaw = math.atan2(math.sin(cur_yaw - target_yaw), math.cos(cur_yaw - target_yaw))
            e_ct = -(cur_x - target_x) * math.sin(target_yaw) + (cur_y - target_y) * math.cos(target_yaw)
            self.error_log.append([rospy.get_time() - start_time, e_ct, cur_v])

            x0 = np.array([e_ct, cur_v, e_yaw])
            self.ocp_solver.set(0, "lbx", x0)
            self.ocp_solver.set(0, "ubx", x0)

            # 런타임 가중치 및 목표 속도 업데이트
            W_now = np.block([[Q_now, np.zeros((3,2))], [np.zeros((2,3)), R_now]])
            for t in range(self.T):
                self.ocp_solver.cost_set(t, "W", W_now)
                self.ocp_solver.cost_set(t, "yref", np.array([0.0, self.target_v, 0.0, 0.0, 0.0]))
                idx = min(self.closest_idx + int(t * cur_v * self.dt / getattr(self, 'path_resolution', 0.1)), len(self.ckappa)-1)
                self.ocp_solver.set(t, "p", np.array([self.ckappa[idx]]))

            status = self.ocp_solver.solve()

            if status == 0:
                u0 = self.ocp_solver.get(0, "u")
                accel_cmd, steer_cmd = float(u0[0]), float(u0[1])
                if accel_cmd >= 0.0: accel, brake = accel_cmd, 0.0
                else:
                    accel = 0.0
                    brake = 0.0 if accel_cmd > -0.3 else min(-accel_cmd - 0.3, getattr(self, 'max_brake', self.max_accel))
            else:
                rospy.logwarn_throttle(1.0, f"acados 풀이 실패(status={status}). Stanley 안전 모드 전환.")
                steer_cmd, accel, brake = self.run_stanley_imported_fallback()

            track_msg = TrackingInfo(cross_track_error=e_ct, heading_error=e_yaw, curvature_gain=self.curve_gain, steering=steer_cmd)
            self.track_info_pub.publish(track_msg)

            ctrl_msg = CtrlCmd(ctrl_mode=2, cmd_type=1, gear=4, steer=steer_cmd,
                               accel=np.clip(accel/self.max_accel, 0.0, 1.0), brake=np.clip(brake/getattr(self, 'max_brake', self.max_accel), 0.0, 1.0))
            self.ctrl_pub.publish(ctrl_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        AcadosMPCController().run()
    except rospy.ROSInterruptException:
        pass