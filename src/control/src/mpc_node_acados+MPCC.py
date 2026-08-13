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

try:
    from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
    import casadi as ca
except ImportError:
    rospy.logerr("acados_template 또는 casadi가 설치되지 않았습니다.")

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

from test_stanley import stanley, state


class UnifiedMPCCAcadosController:
    """
    acados(초고속 NMPC)와 MPCC(윤곽 제어)를 통합한 궁극의 자율주행 제어 노드.
    경로 진행 변수(theta)를 통해 e_c(Contouring)와 e_l(Lag)을 직교 최적화하며,
    RL 적응형 가중치 노드(/mpc_weight)와 실시간 연동됩니다.
    """
    def __init__(self):
        rospy.init_node('mpc_trajectory_follower_unified', anonymous=True)

        package_path = rospkg.RosPack().get_path('control')
        yaml_path = os.path.join(package_path, 'config', 'config.yaml')
        with open(yaml_path, 'r') as f:
            self.__dict__.update(yaml.safe_load(f))

        self.state_lock = threading.Lock()
        self.error_log = deque(maxlen=int(getattr(self, 'error_log_maxlen', 36000)))

        # Stanley 안전 시스템
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

        self.Q = np.diag([self.q_ct_base, self.q_v_base, self.q_yaw_base, self.q_v_base * 0.5])
        self.R = np.diag([self.r_accel_base, self.r_steer_base, self.r_accel_base * 0.1])

        self.path_csv_path = rospy.get_param('~path_csv_path', '/home/acca/acca_ws/global_path/global_path.csv')
        self.load_path_from_csv(self.path_csv_path)

        # acados + MPCC 통합 NMPC 솔버 생성
        self.ocp_solver = self._setup_unified_acados_mpcc_solver()

        rospy.Subscriber('/localization/kinematic_state', Odometry, self.odom_callback)
        rospy.Subscriber('/global_path', Path, self.path_callback)
        rospy.Subscriber('target_velocity', Float32, self.target_vel_callback)
        rospy.Subscriber('/mpc_weight', MPCWeight, self.weight_callback)

        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)
        self.track_info_pub = rospy.Publisher('/tracking_info', TrackingInfo, queue_size=10)

        self.rate = rospy.Rate(1.0 / self.dt)
        rospy.loginfo("acados + MPCC 통합 NMPC 제어 노드가 시작되었습니다.")

    def _setup_unified_acados_mpcc_solver(self):
        """CasADi로 MPCC(Contouring/Lag) 심볼릭 오차 변환 및 acados NMPC를 구축합니다."""
        model = AcadosModel()
        model.name = 'unified_mpcc_acados'

        # 상태 변수: [X, Y, yaw, v, theta(경로 진행거리)]
        X, Y, yaw, v, theta = ca.SX.sym('X'), ca.SX.sym('Y'), ca.SX.sym('yaw'), ca.SX.sym('v'), ca.SX.sym('theta')
        x = ca.vertcat(X, Y, yaw, v, theta)

        # 제어 입력: [accel, steer, v_theta(진행속도 제어)]
        accel, steer, v_theta = ca.SX.sym('accel'), ca.SX.sym('steer'), ca.SX.sym('v_theta')
        u = ca.vertcat(accel, steer, v_theta)

        # 온라인 파라미터: [X_ref, Y_ref, yaw_ref, kappa_ref]
        X_ref, Y_ref, yaw_ref, kappa_ref = ca.SX.sym('X_ref'), ca.SX.sym('Y_ref'), ca.SX.sym('yaw_ref'), ca.SX.sym('kappa_ref')
        p = ca.vertcat(X_ref, Y_ref, yaw_ref, kappa_ref)

        # 비선형 운동학 ODE
        x_dot = ca.SX.sym('x_dot', 5)
        f_expl = ca.vertcat(
            v * ca.cos(yaw),
            v * ca.sin(yaw),
            v / self.wheelbase * ca.tan(steer),
            accel,
            v_theta
        )
        model.f_expl_expr = f_expl
        model.f_impl_expr = x_dot - f_expl
        model.x, model.u, model.xdot, model.p = x, u, x_dot, p

        # MPCC 오차 직교 분리 수식
        dx, dy = X - X_ref, Y - Y_ref
        e_c = -dx * ca.sin(yaw_ref) + dy * ca.cos(yaw_ref)  # Contouring Error
        e_l = -dx * ca.cos(yaw_ref) - dy * ca.sin(yaw_ref)  # Lag Error
        e_yaw = yaw - yaw_ref

        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = self.T
        ocp.solver_options.tf = self.T * self.dt

        # 비용 함수 (e_c^2 + e_l^2 + e_yaw^2 + (v_theta - target_v)^2)
        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.model.cost_y_expr = ca.vertcat(e_c, e_l, e_yaw, v_theta, accel, steer, v_theta)
        ocp.model.cost_y_expr_e = ca.vertcat(e_c, e_l, e_yaw, v_theta)

        ocp.cost.W = np.block([
            [self.Q, np.zeros((4, 3))],
            [np.zeros((3, 4)), self.R]
        ])
        ocp.cost.W_e = np.diag([self.qf_ct, self.qf_v, self.qf_yaw, self.qf_v * 0.5])
        ocp.cost.yref = np.zeros(7)
        ocp.cost.yref_e = np.zeros(4)

        # 제약 조건 및 타이어 마찰 원(Friction Circle) 한계 설정
        ocp.constraints.lbu = np.array([-self.max_accel, -self.max_steer, 0.0])
        ocp.constraints.ubu = np.array([self.max_accel, self.max_steer, self.max_speed])
        ocp.constraints.idxbu = np.array([0, 1, 2])
        ocp.constraints.lbx = np.array([-np.inf, -np.inf, -np.inf, self.min_speed, 0.0])
        ocp.constraints.ubx = np.array([np.inf, np.inf, np.inf, self.max_speed, np.inf])
        ocp.constraints.idxbx = np.array([0, 1, 2, 3, 4])
        ocp.constraints.x0 = np.zeros(5)
        ocp.parameter_values = np.zeros(4)

        # HPIPM 기반 SQP RTI 초고속 연산 옵션
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.qp_solver_cond_N = self.T

        return AcadosOcpSolver(ocp, json_file='acados_mpcc_ocp.json')

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
            self.curve_gain = 0.0; return
        end_idx = min(self.closest_idx + self.curve_preview, len(self.ckappa))
        future = np.abs(self.ckappa[self.closest_idx : end_idx])
        if len(future) == 0: self.curve_gain = 0.0; return
        weights = np.exp(-0.3 * np.arange(len(future)))
        self.curve_gain = np.sum(future * weights) / np.sum(weights)

    def weight_callback(self, msg):
        with self.state_lock:
            self.Q[0,0], self.Q[1,1], self.Q[2,2] = msg.q_ct, msg.q_v, msg.q_yaw
            self.R[0,0], self.R[1,1] = msg.r_accel, msg.r_steer

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
                Q_now, R_now = self.Q, self.R

            target_x, target_y, target_yaw = self.find_closest_waypoint(cur_x, cur_y)
            if target_x is None:
                self.rate.sleep(); continue

            self.update_curve_gain()
            e_yaw = math.atan2(math.sin(cur_yaw - target_yaw), math.cos(cur_yaw - target_yaw))
            dx, dy = cur_x - target_x, cur_y - target_y
            e_c = -dx * math.sin(target_yaw) + dy * math.cos(target_yaw)
            self.error_log.append([rospy.get_time() - start_time, e_c, cur_v])

            # 초기 상태 주입 [X, Y, yaw, v, theta]
            x0 = np.array([cur_x, cur_y, cur_yaw, cur_v, 0.0])
            self.ocp_solver.set(0, "lbx", x0); self.ocp_solver.set(0, "ubx", x0)

            W_now = np.block([[Q_now, np.zeros((4,3))], [np.zeros((3,4)), R_now]])
            for t in range(self.T):
                self.ocp_solver.cost_set(t, "W", W_now)
                self.ocp_solver.cost_set(t, "yref", np.array([0.0, 0.0, 0.0, self.target_v, 0.0, 0.0, self.target_v]))
                idx = min(self.closest_idx + int(t * cur_v * self.dt / getattr(self, 'path_resolution', 0.1)), len(self.cx)-1)
                self.ocp_solver.set(t, "p", np.array([self.cx[idx], self.cy[idx], self.cyaw[idx], self.ckappa[idx]]))

            status = self.ocp_solver.solve()

            if status == 0:
                u0 = self.ocp_solver.get(0, "u")
                accel_cmd, steer_cmd = float(u0[0]), float(u0[1])
                if accel_cmd >= 0.0: accel, brake = accel_cmd, 0.0
                else:
                    accel = 0.0
                    brake = 0.0 if accel_cmd > -0.3 else min(-accel_cmd - 0.3, getattr(self, 'max_brake', self.max_accel))
            else:
                rospy.logwarn_throttle(1.0, f"통합 NMPC 풀이 실패(status={status}). Stanley 안전 모드 전환.")
                steer_cmd, accel, brake = self.run_stanley_imported_fallback()

            track_msg = TrackingInfo(cross_track_error=e_c, heading_error=e_yaw, curvature_gain=self.curve_gain, steering=steer_cmd)
            self.track_info_pub.publish(track_msg)

            ctrl_msg = CtrlCmd(ctrl_mode=2, cmd_type=1, gear=4, steer=steer_cmd,
                               accel=np.clip(accel/self.max_accel, 0.0, 1.0), brake=np.clip(brake/getattr(self, 'max_brake', self.max_accel), 0.0, 1.0))
            self.ctrl_pub.publish(ctrl_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        UnifiedMPCCAcadosController().run()
    except rospy.ROSInterruptException:
        pass