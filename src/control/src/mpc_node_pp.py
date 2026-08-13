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
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, Float32MultiArray
from morai_msgs.msg import CtrlCmd, TrackingInfo, MPCWeight
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from scipy.ndimage import gaussian_filter1d
import hpipm_python as hpipm

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

#스탠리
from test_stanley import stanley, state
from pure_pursuit import pure_pursuit

class MPCController:
    def __init__(self):
        rospy.init_node('mpc_trajectory_follower', anonymous=True)

        package_path = rospkg.RosPack().get_path('control')
        yaml_candidates = [
            os.path.join(package_path, 'config', 'config.yaml'),
            os.path.join(package_path, 'src', 'config.yaml'),
            os.path.join(package_path, 'parameter', 'config.yaml'),
        ]
        yaml_path = next((path for path in yaml_candidates if os.path.exists(path)), None)
        if yaml_path is None:
            raise FileNotFoundError("MPC config.yaml not found in: %s" % yaml_candidates)

        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        self.__dict__.update(config)

        # === [데이터 로깅 세팅] ===
        self.error_log = []  
        # self.csv_path = os.path.join('/home/acca/acca_ws/global_path/global_path.csv')
        # ==========================

        # 임포트해 온 클래스로 안전하게 인스턴스 객체 할당 완료
        self.stanley_solver = stanley()
        
        self.stanley_solver.L = self.wheelbase  # 축거 데이터 동기화
        self.stanley_solver.kv = self.stanley_kv  # yaml에서 불러온 저속 댐핑 안정화 게인 값 동기화

        self.pp_solver = pure_pursuit()
        self.pp_state = state()
        self.pp_solver.WB = self.wheelbase
        self.pp_solver.k = self.k
        
        self.odom_received = False ##\\##

        self.cx = []  
        self.cy = []
        self.cyaw = []

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_v = 0.0
        self.current_yaw = 0.0
        self.path_topic = rospy.get_param('~path_topic', '/global_path')
        self.target_v = 0.0 if self.path_topic != '/global_path' else 11.5
        self.ai_path_timeout = float(rospy.get_param('~ai_path_timeout', 0.5))
        self.stop_full_brake_threshold = float(
            rospy.get_param('~stop_full_brake_threshold', 0.8)
        )
        if not 0.0 <= self.stop_full_brake_threshold <= 1.0:
            raise ValueError("stop_full_brake_threshold must be in [0, 1]")
        self.stop_latch_threshold = float(
            rospy.get_param('~stop_latch_threshold', 0.5)
        )
        self.stop_release_drive_threshold = float(
            rospy.get_param('~stop_release_drive_threshold', 0.8)
        )
        self.stop_release_hold_sec = float(
            rospy.get_param('~stop_release_hold_sec', 2.0)
        )
        if not 0.0 <= self.stop_latch_threshold <= 1.0:
            raise ValueError("stop_latch_threshold must be in [0, 1]")
        if not 0.0 <= self.stop_release_drive_threshold <= 1.0:
            raise ValueError("stop_release_drive_threshold must be in [0, 1]")
        if self.stop_release_hold_sec < 0.0:
            raise ValueError("stop_release_hold_sec cannot be negative")
        self.mode_score_timeout = float(
            rospy.get_param('~mode_score_timeout', self.ai_path_timeout)
        )
        self.stop_probability = 0.0
        self.drive_probability = 0.0
        self.model_stop_latched = False
        self.model_full_brake_latched = False
        self.stop_release_started = None
        self.last_mode_score_received = None
        self.last_path_received = None
        self.last_ai_path_stamp = None
        self.global_path = [] 
        self.path_csv_path = rospy.get_param('~path_csv_path', '/home/acca/acca_ws/global_path/global_path.csv')

        # Path-dependent state must exist before load_path_from_csv() computes
        # curvature around the current closest waypoint.
        self.closest_idx = 0
        self.curve_gain = 0.0
        self.ckappa = np.array([], dtype=float)
        self.path_s = np.array([], dtype=float)

        self.v_error_integral = 0.0
        self.prev_accel_cmd = 0.0

        # 실제 명령에 적용된 직전 입력입니다. MPC 출력의 급격한 반전을 제한하는 데 사용합니다.
        self.last_accel_cmd = 0.0
        self.last_steer_cmd = 0.0

        # YAML에 값이 있으면 그 값을 사용하고, 없으면 보수적인 기본값을 사용합니다.
        # 단위: max_accel [m/s^2], max_steer_rate [rad/s]
        self.max_accel_rate = float(getattr(self, 'max_accel', 2.0))
        self.max_steer_rate = float(
            getattr(self, 'max_dsteer', math.radians(60.0))
        )
        self.mpc_log_period = float(getattr(self, 'mpc_log_period', 0.1))
      
        self.Q = np.diag([self.q_ct_base, self.q_v_base, self.q_yaw_base])        
        self.Qf = np.diag([self.qf_ct, self.qf_v, self.qf_yaw])
        self.R = np.diag([self.r_accel_base, self.r_steer_base])
        #self.Rd = np.diag([self.rd_accel, self.rd_steer])
       
        self.lbu = np.array([-self.max_accel, -self.max_steer]) 
        self.ubu = np.array([self.max_accel, self.max_steer])   

        self.lbx = np.array([-np.inf, self.min_speed, -np.inf]) 
        self.ubx = np.array([np.inf, self.max_speed, np.inf])   

        if self.path_topic == '/global_path':
            self.load_path_from_csv(self.path_csv_path)
        else:
            rospy.loginfo(
                "AI path mode: CSV preload disabled, waiting for %s",
                self.path_topic
            )

        self.prev_accel = np.zeros(self.T, dtype=float)
        self.prev_delta = np.zeros(self.T, dtype=float)

        # Run one control update for each new localization sample. This avoids
        # an arbitrary phase offset between two independent 10 Hz loops and
        # prevents the MPC from repeatedly correcting a stale vehicle state.
        self.odom_event = threading.Event()
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)
        self.track_info_pub = rospy.Publisher(
            '/tracking_info', TrackingInfo, queue_size=10
        )
        rospy.Subscriber('/localization/kinematic_state', Odometry, self.odom_callback)
        rospy.Subscriber(self.path_topic, Path, self.path_callback)
        self.target_velocity_topic = rospy.get_param(
            '~target_velocity_topic', 'target_velocity'
        )
        rospy.Subscriber(
            self.target_velocity_topic, Float32, self.target_vel_callback
        )
        rospy.Subscriber(
            rospy.get_param(
                '~mode_score_topic',
                '/multimodal_learning/mode_scores'
            ),
            Float32MultiArray,
            self.mode_score_callback,
            queue_size=1,
        )
        rospy.Subscriber('/mpc_weight', MPCWeight, self.weight_callback)

        self.rate = rospy.Rate(10)
        rospy.loginfo("MPC 제어 노드가 성공적으로 시작되었습니다.")

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.current_yaw = yaw

        # /localization/kinematic_state is located at the rear-axle center,
        # which is also the reference point of the kinematic bicycle model.
        # Use it directly without a forward wheelbase offset.
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_v = msg.twist.twist.linear.x

        self.odom_received = True ##\\##
        self.odom_event.set()

    def path_callback(self, msg):
        if self.path_topic != '/global_path':
            path_stamp = msg.header.stamp.to_nsec()
            if path_stamp == self.last_ai_path_stamp:
                self.last_path_received = rospy.Time.now()
                return
            self.last_ai_path_stamp = path_stamp

        if msg.header.frame_id in ('base_link', 'base_footprint'):
            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            self.global_path = [
                (
                    self.current_x
                    + cos_yaw * pose.pose.position.x
                    - sin_yaw * pose.pose.position.y,
                    self.current_y
                    + sin_yaw * pose.pose.position.x
                    + cos_yaw * pose.pose.position.y,
                )
                for pose in msg.poses
            ]
            self.closest_idx = 0
            self._initialized_search = False
        else:
            self.global_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
            if self.path_topic != '/global_path':
                self.closest_idx = 0
                self._initialized_search = False
        if len(self.global_path) < 2:
            return
        self.cx = np.array([pt[0] for pt in self.global_path])
        self.cy = np.array([pt[1] for pt in self.global_path])

        if self.path_topic != '/global_path':
            path_yaws = []
            for pose in msg.poses:
                orientation = pose.pose.orientation
                _, _, path_yaw = euler_from_quaternion([
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                ])
                if msg.header.frame_id in ('base_link', 'base_footprint'):
                    path_yaw += self.current_yaw
                path_yaws.append(
                    math.atan2(math.sin(path_yaw), math.cos(path_yaw))
                )
            self.cyaw = path_yaws
        else:
            self.rebuild_path_yaw()
        self.rebuild_path_distance()
        
        self.ckappa = self.calc_curvature(self.cx, self.cy)
        self.ckappa = gaussian_filter1d(self.ckappa, sigma=1)
        self.last_path_received = rospy.Time.now()

    
    def update_curve_gain(self):
        """현재 차량 위치(closest_idx) 기준으로 미래 예측 Horizon의 곡률 가중평균을 실시간 갱신합니다."""
        if self.ckappa is None or len(self.ckappa) == 0:
            self.curve_gain = 0.0
            return

        # 현재 인덱스부터 제어 Horizon T만큼의 미래 곡률 슬라이싱
        end_idx = min(self.closest_idx + self.T, len(self.ckappa))
        future = np.abs(self.ckappa[self.closest_idx : end_idx])
        
        n_points = len(future)
        if n_points == 0:
            self.curve_gain = 0.0
            return

        # 예측 거리에 따른 지수 감쇄 가중치 생성 (남은 경로가 T보다 짧을 때를 대비해 동적 크기 조절)
        weights = np.exp(-0.3 * np.arange(n_points))
        weights /= np.sum(weights)

        curve_gain = np.sum(future * weights)

        return curve_gain

    def weight_callback(self, msg):
        self.q_ct = msg.q_ct
        self.q_v = msg.q_v
        self.q_yaw = msg.q_yaw

        self.r_accel = msg.r_accel
        self.r_steer = msg.r_steer

        self.Q = np.diag([self.q_ct, self.q_v, self.q_yaw])
        self.R = np.diag([self.r_accel, self.r_steer])

    def load_path_from_csv(self, csv_path):
        if not csv_path:
            rospy.logwarn("path_csv_path 파라미터가 비어 있습니다. /global_path 토픽을 기다립니다.")
            return

        if not os.path.exists(csv_path):
            rospy.logwarn("CSV 경로 파일이 없습니다: %s. /global_path 토픽을 기다립니다.", csv_path)
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

            if len(xs) < 2:
                rospy.logwarn("CSV path point가 부족합니다: %s", csv_path)
                return

            self.cx = np.array(xs)
            self.cy = np.array(ys)
            self.global_path = list(zip(self.cx, self.cy))
            if len(yaws) == len(xs):
                self.cyaw = yaws
            else:
                self.rebuild_path_yaw()
            self.rebuild_path_distance()

            self.ckappa = self.calc_curvature(self.cx, self.cy)
            self.ckappa = gaussian_filter1d(self.ckappa, sigma=3)

            rospy.loginfo("CSV global path 로드 완료: %s (%d points)", csv_path, len(self.cx))
        except Exception as e:
            rospy.logerr("CSV global path 로드 실패: %s (%s)", csv_path, e)
        

    def rebuild_path_yaw(self):
        self.cyaw = []
        for i in range(len(self.cx) - 1):
            dx = self.cx[i+1] - self.cx[i]
            dy = self.cy[i+1] - self.cy[i]
            self.cyaw.append(math.atan2(dy, dx))
        
        if len(self.cyaw) > 0:
            self.cyaw.append(self.cyaw[-1])

    def rebuild_path_distance(self):
        if len(self.cx) == 0:
            self.path_s = np.array([], dtype=float)
            return
        segment_length = np.hypot(np.diff(self.cx), np.diff(self.cy))
        self.path_s = np.concatenate(([0.0], np.cumsum(segment_length)))

    def target_vel_callback(self, msg):
        self.target_v = float(np.clip(msg.data, 0.0, self.max_speed))

    def mode_score_callback(self, msg):
        if len(msg.data) < 2:
            rospy.logwarn_throttle(
                2.0, "Mode score message needs [STOP, DRIVE] probabilities"
            )
            return
        stop_probability = float(msg.data[0])
        drive_probability = float(msg.data[1])
        if not np.isfinite(stop_probability) or not np.isfinite(
            drive_probability
        ):
            rospy.logwarn_throttle(2.0, "STOP/DRIVE probability is not finite")
            return
        self.stop_probability = float(np.clip(stop_probability, 0.0, 1.0))
        self.drive_probability = float(np.clip(drive_probability, 0.0, 1.0))
        now = rospy.Time.now()
        self.last_mode_score_received = now

        if not self.model_stop_latched:
            if self.stop_probability >= self.stop_latch_threshold:
                self.model_stop_latched = True
                self.stop_release_started = None
                rospy.logwarn(
                    "Model STOP latched: STOP probability %.3f",
                    self.stop_probability,
                )
        else:
            if self.drive_probability >= self.stop_release_drive_threshold:
                if self.stop_release_started is None:
                    self.stop_release_started = now
                elif (
                    now - self.stop_release_started
                ).to_sec() >= self.stop_release_hold_sec:
                    self.model_stop_latched = False
                    self.model_full_brake_latched = False
                    self.stop_release_started = None
                    rospy.loginfo(
                        "Model STOP released after DRIVE probability %.3f "
                        "held for %.2fs",
                        self.drive_probability,
                        self.stop_release_hold_sec,
                    )
            else:
                self.stop_release_started = None

        if (
            self.model_stop_latched
            and self.stop_probability >= self.stop_full_brake_threshold
        ):
            self.model_full_brake_latched = True
            self.target_v = 0.0
            self.publish_model_full_brake()

    def full_brake_stop_active(self):
        return self.model_full_brake_latched

    def publish_safe_stop(self):
        ctrl_msg = CtrlCmd()
        ctrl_msg.ctrl_mode = 2
        ctrl_msg.cmd_type = 1
        ctrl_msg.gear = 4
        ctrl_msg.steer = 0.0
        ctrl_msg.accel = 0.0
        ctrl_msg.brake = 1.0
        self.ctrl_pub.publish(ctrl_msg)

    def publish_model_full_brake(self):
        ctrl_msg = CtrlCmd()
        ctrl_msg.ctrl_mode = 2
        ctrl_msg.cmd_type = 1
        ctrl_msg.gear = 4
        ctrl_msg.steer = float(np.clip(
            self.last_steer_cmd, -self.max_steer, self.max_steer
        ))
        ctrl_msg.accel = 0.0
        ctrl_msg.brake = 1.0
        self.ctrl_pub.publish(ctrl_msg)

    def find_closest_waypoint(self, x, y):
        if len(self.cx) < 2 or len(self.cyaw) == 0:
            return None, None, None
        
        # 탐색 범위를 한정할 윈도우 크기 설정
        search_range = 50 
        
        # 초기 루프 혹은 강제 전체 검색이 필요한 경우 전체 탐색 범위 지정
        if not hasattr(self, '_initialized_search') or not self._initialized_search:
            start_idx = 0
            end_idx = len(self.cx)
            self._initialized_search = True
        else:
            start_idx = max(0, self.closest_idx - search_range)
            end_idx = min(len(self.cx), self.closest_idx + search_range)
            
        min_dist = float('inf')
        best_idx = self.closest_idx
        
        for i in range(start_idx, end_idx):
            dist = math.sqrt((x - self.cx[i])**2 + (y - self.cy[i])**2)
            if dist < min_dist:
                min_dist = dist
                best_idx = i
                
        # [Fallback] 만약 로컬 윈도우 내 최소 거리가 5.0m 이상이면 
        # 차량이 윈도우를 벗어난 것으로 판단하고 전체 재탐색을 수행해 복구합니다.
        if min_dist > 5.0: ##\\##and start_idx > 0:
            min_dist = float('inf')
            for i in range(len(self.cx)):
                dist = math.sqrt((x - self.cx[i])**2 + (y - self.cy[i])**2)
                if dist < min_dist:
                    min_dist = dist
                    best_idx = i

        self.closest_idx = best_idx
        target_x = self.cx[self.closest_idx]
        target_y = self.cy[self.closest_idx]
        target_yaw = self.cyaw[self.closest_idx] 
        
        return target_x, target_y, target_yaw
    
    def run_stanley_imported_fallback(self):

        self.stanley_state = state(self.current_x, self.current_y, self.current_yaw, self.current_v)

        steer = self.stanley_solver.stan_control(self.stanley_state, self.cx, self.cy, self.cyaw, self.stanley_h_gain, self.stanley_c_gain)

        # target_v = self.target_v * 0.7
        target_v = self.target_v

        v_error = target_v - self.current_v
        
        if not hasattr(self, 'v_error_integral'):
            self.v_error_integral = 0.0
            self.prev_accel_cmd = 0.0
            
        self.v_error_integral += v_error * self.dt
        
        self.v_error_integral = np.clip(self.v_error_integral, -2.0, 2.0)
        
        accel_cmd = (self.kp * v_error) + (self.ki * self.v_error_integral)

        max_u_change = 2.0 * self.dt
        accel_cmd = np.clip(accel_cmd, self.prev_accel_cmd - max_u_change, self.prev_accel_cmd + max_u_change)
        self.prev_accel_cmd = accel_cmd

        if accel_cmd > 0:
            accel = np.clip(accel_cmd, 0.0, self.max_accel)
            brake = 0.0
        else:
            accel = 0.0
            brake = np.clip(-accel_cmd, 0.0, self.max_accel)

        return steer, accel, brake

    def run_pp_imported_fallback(self):

        self.pp_state = state(self.current_x, self.current_y, self.current_yaw, self.current_v)

        steer = self.pp_solver.pure_pursuit_control(self.pp_state, self.cx, self.cy)

        target_v = self.target_v

        v_error = target_v - self.current_v
                
        if not hasattr(self, 'v_error_integral'):
            self.v_error_integral = 0.0
            self.prev_accel_cmd = 0.0
                    
        self.v_error_integral += v_error * self.dt
                
        self.v_error_integral = np.clip(self.v_error_integral, -2.0, 2.0)
                
        accel_cmd = (self.kp * v_error) + (self.ki * self.v_error_integral)
        
        max_u_change = 2.0 * self.dt
        accel_cmd = np.clip(accel_cmd, self.prev_accel_cmd - max_u_change, self.prev_accel_cmd + max_u_change)
        self.prev_accel_cmd = accel_cmd
        
        if accel_cmd > 0:
            accel = np.clip(accel_cmd, 0.0, self.max_accel)
            brake = 0.0
        else:
            accel = 0.0
            brake = np.clip(-accel_cmd, 0.0, self.max_accel)
        
        return steer, accel, brake
    
    def _limit_input_sequence(self, u_seq):
        """MPC 입력 시퀀스에 가속도/조향 변화율 제한을 순차적으로 적용합니다."""
        u_seq = np.asarray(u_seq, dtype=float)

        if u_seq.shape != (self.T, self.nu):
            raise ValueError(
                "Invalid MPC input sequence shape: %s, expected (%d, %d)"
                % (u_seq.shape, self.T, self.nu)
            )

        limited = np.zeros_like(u_seq)
        prev_accel = float(self.last_accel_cmd)
        prev_steer = float(self.last_steer_cmd)

        max_accel_change = self.max_accel_rate * self.dt
        max_steer_change = self.max_steer_rate * self.dt

        for t in range(self.T):
            raw_accel = float(u_seq[t, 0])
            raw_steer = float(u_seq[t, 1])

            accel = np.clip(
                raw_accel,
                prev_accel - max_accel_change,
                prev_accel + max_accel_change
            )
            steer = np.clip(
                raw_steer,
                prev_steer - max_steer_change,
                prev_steer + max_steer_change
            )

            accel = float(np.clip(accel, -self.max_accel, self.max_accel))
            steer = float(np.clip(steer, -self.max_steer, self.max_steer))

            limited[t, 0] = accel
            limited[t, 1] = steer

            prev_accel = accel
            prev_steer = steer

        return limited

    def solve_mpc(self, x0, A_list, B_list, b_list, Q_list, R_list, q_list, r_list):

        dim = hpipm.hpipm_ocp_qp_dim(self.T)
        dim.set('nx', self.nx, 0, self.T)
        dim.set('nu', self.nu, 0, self.T-1)

        dim.set('nbx', self.nx, 0)
        dim.set('nbu', self.nu, 0, self.T-1)

        qp = hpipm.hpipm_ocp_qp(dim)

        for t in range(self.T):
            qp.set('A', A_list[t], t)
            qp.set('B', B_list[t], t)
            qp.set('b', b_list[t], t)
            qp.set('Q', Q_list[t], t)
            qp.set('R', R_list[t], t)
            qp.set('q', q_list[t], t)
            qp.set('r', r_list[t], t)

            qp.set('Jbu', np.eye(self.nu), t)
            qp.set('lbu', self.lbu, t)
            qp.set('ubu', self.ubu, t)

            if t > 0:
                qp.set('Jbx', np.eye(self.nx), t)
                qp.set('lbx', self.lbx, t)
                qp.set('ubx', self.ubx, t)

        qp.set('Q', Q_list[self.T], self.T)
        qp.set('q', q_list[self.T], self.T)

        qp.set('Jbx', np.eye(self.nx), self.T)
        qp.set('lbx', self.lbx, self.T)
        qp.set('ubx', self.ubx, self.T)

        qp.set('Jbx', np.eye(self.nx), 0)
        qp.set('lbx', x0, 0)
        qp.set('ubx', x0, 0)

        qp_sol = hpipm.hpipm_ocp_qp_sol(dim)
        mode = 'speed'
        arg = hpipm.hpipm_ocp_qp_solver_arg(dim, mode)

        solver = hpipm.hpipm_ocp_qp_solver(dim, arg)
        solver.solve(qp, qp_sol)

        status = int(solver.get('status'))

        status = 1

        if status == 0:
            raw_u_seq = np.zeros((self.T, self.nu), dtype=float)

            for t in range(self.T):
                u_t = np.asarray(
                    qp_sol.get('u', t),
                    dtype=float
                ).reshape(-1)

                if u_t.size != self.nu:
                    raise ValueError(
                        "Unexpected HPIPM input shape at t=%d: %s, expected %d elements"
                        % (t, u_t.shape, self.nu)
                    )

                if not np.all(np.isfinite(u_t)):
                    raise ValueError(
                        "HPIPM returned non-finite input at t=%d: %s"
                        % (t, u_t)
                    )

                raw_u_seq[t, :] = u_t

            # QP 해 자체에 입력 변화율 제약이 없으므로 실제 적용 시퀀스에는
            # 가속도 및 조향 변화율 제한을 적용합니다.
            u_seq = self._limit_input_sequence(raw_u_seq)

            self.prev_accel[:-1] = u_seq[1:, 0]
            self.prev_accel[-1] = u_seq[-1, 0]

            self.prev_delta[:-1] = u_seq[1:, 1]
            self.prev_delta[-1] = u_seq[-1, 1]

            raw_accel_cmd = float(raw_u_seq[0, 0])
            raw_steer_cmd = float(raw_u_seq[0, 1])

            accel_cmd = float(u_seq[0, 0])
            steer_cmd = float(u_seq[0, 1])

            self.last_accel_cmd = accel_cmd
            self.last_steer_cmd = steer_cmd

            # rospy.loginfo_throttle(
            #     self.mpc_log_period,
            #     "MPC raw steer=%.4f accel=%.4f | applied steer=%.4f accel=%.4f",
            #     math.degrees(raw_steer_cmd),
            #     raw_accel_cmd,
            #     math.degrees(steer_cmd),
            #     accel_cmd
            # )

            max_acc = float(self.max_accel)
            max_brk = float(getattr(self, 'max_brake', max_acc))

            # 2. 부드러운 감속을 위한 임계값(Threshold) 및 데드밴드 설정
            # 물리 브레이크를 밟기 시작할 최소 감속도 기준 (단위: m/s^2)
            # -0.3 m/s^2 보다 가벼운 감속 요구 시에는 브레이크를 전혀 밟지 않고 액셀만 off 합니다.
            brake_deadband = -0.3 

            if accel_cmd >= 0.0:
                # 가속 시
                accel = float(np.clip(accel_cmd / max_acc, 0.0, 1.0))
                brake = 0.0
            else:
                # 감속 시 (accel_cmd < 0)
                accel = 0.0
                
                if accel_cmd > brake_deadband:
                    # 가벼운 감속 구간: 브레이크를 밟지 않고 타성 주행(엔진 브레이크) 유도
                    brake = 0.0
                else:
                    # 일정 수준 이상의 감속 요구 시에만 브레이크 작동
                    # deadband를 초과한 양에 비례해서 브레이크 압력을 선형적으로 증가시킵니다.
                    brake_ratio = (brake_deadband - accel_cmd) / max_brk
                    brake = float(np.clip(brake_ratio, 0.0, 1.0))
            

            return steer_cmd, accel, brake

        rospy.logwarn_throttle(
            0.1,
            "HPIPM solve failed (status=%d). Stanley fallback is active.",
            status
        )

        # steer, accel, brake = self.run_stanley_imported_fallback()

        steer, accel, brake = self.run_pp_imported_fallback()

        # fallback 출력도 동일한 변화율 제한을 통과시킵니다.
        signed_accel = float(accel - brake)
        fallback_seq = np.zeros((self.T, self.nu), dtype=float)
        fallback_seq[:, 0] = signed_accel
        fallback_seq[:, 1] = float(steer)
        fallback_seq = self._limit_input_sequence(fallback_seq)

        # 이력 시퀀스 업데이트 필수!
        self.prev_accel[:-1] = fallback_seq[1:, 0]
        self.prev_accel[-1] = fallback_seq[-1, 0]
        self.prev_delta[:-1] = fallback_seq[1:, 1]
        self.prev_delta[-1] = fallback_seq[-1, 1]

        signed_accel = float(fallback_seq[0, 0])
        steer = float(fallback_seq[0, 1])

        self.last_accel_cmd = signed_accel
        self.last_steer_cmd = steer

        if signed_accel >= 0.0:
            accel = float(np.clip(
                signed_accel / self.max_accel, 0.0, 1.0
            ))
            brake = 0.0
        else:
            accel = 0.0
            max_brake = float(getattr(
                self, 'max_brake', self.max_accel
            ))
            brake = float(np.clip(
                -signed_accel / max_brake, 0.0, 1.0
            ))

        return steer, accel, brake

    def get_state_space_matrices(self, v, e_yaw, delta, kappa):
        """비선형 오차 모델을 현재 nominal state/input 주변에서 선형화합니다."""
        v = max(0.1, float(v))
        e_yaw = float(e_yaw)
        delta = float(np.clip(delta, -self.max_steer, self.max_steer))
        kappa = float(kappa)

        A = np.eye(self.nx)
        B = np.zeros((self.nx, self.nu))
        b = np.zeros(self.nx)

        sin_yaw = math.sin(e_yaw)
        cos_yaw = math.cos(e_yaw)
        tan_delta = math.tan(delta)
        cos_delta = math.cos(delta)
        sec2_delta = 1.0 / max(cos_delta ** 2, 1e-8)

        # e_ct(k+1) = e_ct + v * sin(e_yaw) * dt
        A[0, 1] = sin_yaw * self.dt
        A[0, 2] = v * cos_yaw * self.dt

        # v(k+1) = v + a * dt
        B[1, 0] = self.dt

        # e_yaw(k+1)
        # = e_yaw + (v / L * tan(delta) - v * kappa) * dt
        A[2, 1] = (
            tan_delta / self.wheelbase - kappa
        ) * self.dt

        B[2, 1] = (
            v / self.wheelbase * sec2_delta
        ) * self.dt

        # f(x_bar, u_bar) - A*x_bar - B*u_bar
        b[0] = -v * e_yaw * cos_yaw * self.dt
        b[2] = (
            -v * delta / self.wheelbase * sec2_delta * self.dt
        )

        return A, B, b

    def calc_curvature(self, x, y):
        
        dx = np.gradient(x)
        dy = np.gradient(y)

        ddx = np.gradient(dx)
        ddy = np.gradient(dy)

        denominator = np.maximum((dx**2 + dy**2)**1.5, 1e-8)

        kappa = (dx * ddy - dy * ddx) / denominator

        return kappa

    def distance_to_index(self, distance):
        if len(self.path_s) != len(self.ckappa):
            step = int(round(distance / self.path_resolution))
            return min(self.closest_idx + step, len(self.ckappa) - 1)

        target_s = self.path_s[self.closest_idx] + max(float(distance), 0.0)
        return min(
            int(np.searchsorted(self.path_s, target_s, side='left')),
            len(self.ckappa) - 1
        )

    def predict_motion(self, x0, oa, odelta):

        if self.ckappa is None or len(self.ckappa) == 0:
            rospy.logwarn_throttle(2.0, "[MPC] 글로벌 경로(ckappa) 데이터가 비어있어 예측을 건너뜁니다.")
            return None
        """선형화 기준점 생성을 위한 비선형 오차 모델 예측입니다."""
        xbar = np.zeros((self.nx, self.T + 1), dtype=float)
        xbar[:, 0] = x0

        x = np.asarray(x0, dtype=float).copy()

        predicted_distance = 0.0

        for t in range(self.T):
            e_ct = float(x[0])
            v = float(x[1])
            e_yaw = float(x[2])

            a = float(oa[t])
            delta = float(np.clip(
                odelta[t],
                -self.max_steer,
                self.max_steer
            ))

            idx = self.distance_to_index(predicted_distance)

            kappa = float(self.ckappa[idx])

            e_ct_next = (
                e_ct + v * math.sin(e_yaw) * self.dt
            )

            v_next = v + a * self.dt

            e_yaw_next = (
                e_yaw
                + (
                    v / self.wheelbase * math.tan(delta)
                    - v * kappa
                ) * self.dt
            )
            e_yaw_next = math.atan2(
                math.sin(e_yaw_next),
                math.cos(e_yaw_next)
            )

            x = np.array([
                e_ct_next,
                v_next,
                e_yaw_next
            ], dtype=float)

            xbar[:, t + 1] = x

            average_v = max(0.5 * (v + v_next), 0.0)
            predicted_distance += average_v * self.dt
            # rospy.loginfo_throttle(0.05, f'predicted_distance : {predicted_distance}')
        return xbar

    def prepare_mpc_matrices(self, xref, xbar):
        T = self.T 
        A_list, B_list, b_list = [], [], []
        Q_list, R_list, q_list, r_list = [], [], [], []

        predicted_distance = 0.0
        speed_reference = max(
            float(getattr(self, 'speed_weight_reference_kph', 35.0)) / 3.6,
            0.1
        )
        rd_steer_base = float(getattr(self, 'r_dsteer_base', 0.0))
        k_speed_dsteer = float(getattr(self, 'k_speed_dsteer', 0.0))
        dsteer_speed_exponent = max(
            float(getattr(self, 'dsteer_speed_exponent', 1.0)), 1.0
        )
        dsteer_high_speed_kph = float(
            getattr(self, 'dsteer_high_speed_kph', 45.0)
        )
        dsteer_transition_kph = max(
            float(getattr(self, 'dsteer_transition_kph', 5.0)), 0.1
        )
        dsteer_high_speed_gain = float(
            getattr(self, 'dsteer_high_speed_gain', 0.0)
        )

        for t in range(T):

            idx = self.distance_to_index(predicted_distance)

            

            kappa = self.ckappa[idx]

            A_d, B_d, b_d = self.get_state_space_matrices(
                xbar[1, t],
                xbar[2, t],
                self.prev_delta[t],
                kappa
            )

            A_list.append(A_d)
            B_list.append(B_d)
            b_list.append(b_d)

            Q_list.append(self.Q)

            q_step = -1.0 * np.dot(self.Q, xref[:, t])
            r_step = np.zeros(self.nu)

            # Penalize deviation from the previous MPC steering sequence.
            # The penalty rises smoothly with speed and is included in the QP
            # objective instead of clipping the steering command afterwards:
            #   0.5 * rd(v) * (delta - delta_previous)^2
            speed_factor = np.tanh(
                max(float(xbar[1, t]), 0.0) / speed_reference
            )
            rd_steer = rd_steer_base * (
                1.0
                + k_speed_dsteer
                * np.power(speed_factor, dsteer_speed_exponent)
            )
            predicted_speed_kph = max(float(xbar[1, t]), 0.0) * 3.6
            high_speed_factor = np.logaddexp(
                0.0,
                (
                    predicted_speed_kph - dsteer_high_speed_kph
                ) / dsteer_transition_kph
            )
            rd_steer += dsteer_high_speed_gain * high_speed_factor
            R_step = self.R.copy()
            R_step[1, 1] += rd_steer
            r_step[1] = -rd_steer * self.prev_delta[t]
            R_list.append(R_step)

            if t == 0:
                self.current_rd_steer = rd_steer
            
            q_list.append(q_step)
            r_list.append(r_step)

            v0 = xbar[1, t]
            v1 = xbar[1, t + 1]

            average_v = max(0.5 * (v0 + v1), 0.0)
            predicted_distance += average_v * self.dt

        Q_list.append(self.Qf)
        q_list.append(-1.0 * np.dot(self.Qf, xref[:, T]))

        return A_list, B_list, b_list, Q_list, R_list, q_list, r_list

    # def save_data(self):
    #      if not self.error_log:
    #          rospy.logwarn(f"현재 수집된 데이터가 0개입니다.")
    #          return

    #      rospy.loginfo(f"총 {len(self.error_log)}개의 주행 데이터를 엑셀(CSV) 파일로 내보내는 중...")
    #      try:
    #          os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
    #          with open(self.csv_path, 'w', newline='', encoding='utf-8') as f:
    #              writer = csv.writer(f)
    #              writer.writerow(['Time (s)', 'CrossTrack Error (m)', 'Velocity (m/s)'])
    #              writer.writerows(self.error_log)
    #          rospy.loginfo(f"🎉 [파일 저장 성공] 경로: {self.csv_path}")
    #      except Exception as e:
    #          rospy.logerr(f"파일 저장 중 실패: {e}")

    def run(self):
        start_time = rospy.get_time()
        
        while not rospy.is_shutdown():
            # Localization is the control clock: exactly one MPC update is
            # performed after a fresh 10 Hz state sample arrives.
            if not self.odom_event.wait(timeout=0.5):
                self.publish_safe_stop()
                rospy.logwarn_throttle(
                    2.0, "Localization timeout; safe stop active"
                )
                continue
            self.odom_event.clear()

            if self.path_topic != '/global_path':
                path_missing = self.last_path_received is None
                path_stale = (
                    not path_missing
                    and (rospy.Time.now() - self.last_path_received).to_sec()
                    > self.ai_path_timeout
                )
                if path_missing or path_stale:
                    self.target_v = 0.0
                    self.publish_safe_stop()
                    rospy.logwarn_throttle(
                        2.0, "AI path missing or stale; safe stop active"
                    )
                    continue

            if self.full_brake_stop_active():
                self.target_v = 0.0
                self.v_error_integral = 0.0
                self.prev_accel_cmd = 0.0
                self.last_accel_cmd = 0.0
                self.prev_accel.fill(0.0)
                self.publish_model_full_brake()
                rospy.logwarn_throttle(
                    1.0,
                    "STOP full-brake latch active: current STOP %.3f, "
                    "engage threshold %.3f",
                    self.stop_probability,
                    self.stop_full_brake_threshold,
                )
                continue

            if self.model_stop_latched:
                # A brief DRIVE misclassification must not restore the road
                # speed while waiting at a red light. Below the full-brake
                # threshold, keep the ordinary MPC deceleration toward zero.
                self.target_v = 0.0

            target_x, target_y, target_yaw = self.find_closest_waypoint(self.current_x, self.current_y)
            
            if target_x is None:
                rospy.logwarn_throttle(2.0, "글로벌 경로를 기다리는 중입니다...")
                continue
            
            self.curve_gain = self.update_curve_gain()

            rospy.loginfo_throttle(0.1, f'Curve Gain: {self.curve_gain:.4f}, Closest Index: {self.closest_idx}, Path Points: {len(self.cx)}')
            

            e_yaw = self.current_yaw - target_yaw
            e_yaw = math.atan2(math.sin(e_yaw), math.cos(e_yaw))

            dx = self.current_x - target_x
            dy = self.current_y - target_y
            e_ct = -dx * math.sin(target_yaw) + dy * math.cos(target_yaw)

            sim_time = rospy.get_time() - start_time
            self.error_log.append([sim_time, e_ct, self.current_v])

            x0 = np.array([e_ct, self.current_v, e_yaw])

            xref = np.zeros((self.nx, self.T + 1))                      
            xref[1, :] = self.target_v

            xbar = self.predict_motion(
                x0,
                self.prev_accel,
                self.prev_delta
            )

            # xbar = np.zeros((self.nx, self.T + 1))
            # for t in range(self.T + 1):
            #     xbar[:, t] = xref[:, t]  

            A_list, B_list, b_list, Q_list, R_list, q_list, r_list = self.prepare_mpc_matrices(xref, xbar)
            
            steer, accel, brake = self.solve_mpc(
                x0,
                A_list,
                B_list,
                b_list,
                Q_list,
                R_list,
                q_list,
                r_list
            )

            # rospy.loginfo_throttle(
            #     self.mpc_log_period,
            #     "idx=%d e_ct=%.3f e_yaw=%.3f v=%.3f target_v=%.3f",
            #     self.closest_idx,
            #     e_ct,
            #     e_yaw,
            #     self.current_v,
            #     self.target_v
            # )
            
            track_msg = TrackingInfo()
            track_msg.cross_track_error = e_ct
            track_msg.heading_error = e_yaw
            track_msg.curvature_gain = self.curve_gain
            track_msg.steering = steer

            self.track_info_pub.publish(track_msg)


            ctrl_msg = CtrlCmd()
            ctrl_msg.ctrl_mode = 2
            ctrl_msg.cmd_type = 1
            ctrl_msg.gear = 4
            
            ctrl_msg.steer = float(steer)
            # solve_mpc() returns MORAI-normalized pedal commands.
            ctrl_msg.accel = float(np.clip(accel, 0.0, 1.0))
            ctrl_msg.brake = float(np.clip(brake, 0.0, 1.0))

            # A STOP score can arrive while HPIPM is solving. Never allow the
            # just-computed throttle/brake command to overwrite an active
            # full-brake latch.
            if self.full_brake_stop_active():
                self.target_v = 0.0
                self.publish_model_full_brake()
                continue

            self.ctrl_pub.publish(ctrl_msg)

            rospy.loginfo_throttle(
                1.0,
                "MPC weights: Q=%s R=%s Rd_steer(v)=%.2f",
                self.Q,
                self.R,
                float(getattr(self, 'current_rd_steer', 0.0))
            )

if __name__ == '__main__':
    controller = None
    try:
        controller = MPCController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    # finally:
    #      if controller is not None:
    #          controller.save_data()
