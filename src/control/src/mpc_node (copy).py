#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import sys
import math
import numpy as np
import yaml
import csv  
import rospkg
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from morai_msgs.msg import CtrlCmd
from tf.transformations import euler_from_quaternion

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

#스탠리
from test_stanley import stanley, state

class MPCController:
    def __init__(self):
        rospy.init_node('mpc_trajectory_follower', anonymous=True)

        package_path = rospkg.RosPack().get_path('Control')
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
        # self.csv_path = os.path.join(package_path, 'data', 'mpc_log7.csv')
        # ==========================

        # 임포트해 온 클래스로 안전하게 인스턴스 객체 할당 완료
        self.stanley_solver = stanley()
        self.stanley_state = state()
        self.stanley_solver.L = self.wheelbase  # 축거 데이터 동기화
        self.stanley_solver.kv = self.stanley_kv  # yaml에서 불러온 저속 댐핑 안정화 게인 값 동기화
        
        self.cx = []  
        self.cy = []
        self.cyaw = []

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_v = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.last_mpc_error = None
        self.prev_steer_cmd = 0.0
        self.target_v = 10.0
        self.global_path = [] 
        self.path_csv_path = rospy.get_param('~path_csv_path', '/home/acca/global_path.csv')
      
        self.Q = np.diag([self.q_x, self.q_y, self.q_v, self.q_yaw])
        self.Qf = np.diag([self.qf_x, self.qf_y, self.qf_v, self.qf_yaw])
        self.R = np.diag([self.r_accel, self.r_steer])
        #self.Rd = np.diag([self.rd_accel, self.rd_steer])
       
        self.lbu = np.array([-self.max_accel, -self.max_steer]) 
        self.ubu = np.array([self.max_accel, self.max_steer])   

        #self.lbx = np.array([-np.inf, -np.inf, self.min_speed, -np.inf]) 
        #self.ubx = np.array([np.inf, np.inf, self.max_speed, np.inf])   

        self.load_path_from_csv(self.path_csv_path)

        rospy.Subscriber('/localization/kinematic_state', Odometry, self.odom_callback)
        rospy.Subscriber('/global_path', Path, self.path_callback)
        rospy.Subscriber('target_velocity', Float32, self.target_vel_callback)

        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)

        self.rate = rospy.Rate(10)
        rospy.loginfo("MPC 제어 노드가 성공적으로 시작되었습니다.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_v = msg.twist.twist.linear.x
        self.odom_received = True

        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.current_yaw = yaw

    def path_callback(self, msg):
        self.global_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.cx = [pt[0] for pt in self.global_path]
        self.cy = [pt[1] for pt in self.global_path]
        self.cyaw = [0.0] * len(self.cx)

        self.rebuild_path_yaw()

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

            self.cx = xs
            self.cy = ys
            self.global_path = list(zip(self.cx, self.cy))
            if len(yaws) == len(xs):
                self.cyaw = yaws
            else:
                self.rebuild_path_yaw()

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

    def target_vel_callback(self, msg):
        self.target_v = msg.data

    def find_closest_waypoint(self):
        if len(self.cx) < 2 or len(self.cyaw) == 0:
            return None, None, None
        
        min_dist = float('inf')
        closest_idx = 0
        
        for i in range(len(self.cx)):
            dist = math.sqrt((self.current_x - self.cx[i])**2 + (self.current_y - self.cy[i])**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        target_x = self.cx[closest_idx]
        target_y = self.cy[closest_idx]
        target_yaw = self.cyaw[closest_idx] 
        
        return target_x, target_y, target_yaw
    
    def run_stanley_imported_fallback(self):
        """ 불러온 test_stanley 모듈의 객체들을 연동하여 백업 제어 입력을 연산 """
        # 1. 임포트해온 구조체 상태값 동기화
        self.stanley_state.x = self.current_x
        self.stanley_state.y = self.current_y
        self.stanley_state.yaw = self.current_yaw
        self.stanley_state.v = self.current_v

        # 3. 임포트 모듈 함수에 동기화 상태 주입하여 최종 조향각 획득
        steer = self.stanley_solver.stan_control(self.stanley_state, self.cx, self.cy, self.cyaw, self.stanley_h_gain, self.stanley_c_gain)

        # 4. 종방향 엑셀/브레이크 보완 비례 제어 연산
        v_error = self.target_v - self.current_v
        if v_error > 0:
            accel = np.clip(1.0 * v_error, 0.0, self.max_accel)
            brake = 0.0
        else:
            accel = 0.0
            brake = np.clip(-1.0 * v_error, 0.0, self.max_accel)

        return steer, accel, brake
    
    def solve_mpc(self, x0, A_list, B_list, Q_list, R_list, q_list, r_list):
        try:
            import hpipm_python as hpipm
        except ImportError:
            rospy.logwarn_throttle(5.0, "hpipm_python 없음 -> Stanley fallback 사용")
            return self.run_stanley_imported_fallback()

        dim = hpipm.hpipm_ocp_qp_dim(self.T)
        dim.set('nx', self.nx, 0, self.T)      
        dim.set('nu', self.nu, 0, self.T-1)    
        
        dim.set('nbx', self.nx, 0)        
        dim.set('nbu', self.nu, 0, self.T-1)   

        qp = hpipm.hpipm_ocp_qp(dim)
        
        for t in range(self.T):
            qp.set('A', A_list[t], t)
            qp.set('B', B_list[t], t)
            qp.set('Q', Q_list[t], t)
            qp.set('R', R_list[t], t)
            qp.set('q', q_list[t], t)
            qp.set('r', r_list[t], t)
            
            qp.set('Jbu', np.eye(self.nu), t)
            qp.set('lbu', self.lbu, t)
            qp.set('ubu', self.ubu, t)

        qp.set('Q', Q_list[self.T], self.T)
        qp.set('q', q_list[self.T], self.T)

        qp.set('Jbx', np.eye(self.nx), 0)
        qp.set('lbx', x0, 0)
        qp.set('ubx', x0, 0)

        qp_sol = hpipm.hpipm_ocp_qp_sol(dim)
        mode = 'robust'  
        arg = hpipm.hpipm_ocp_qp_solver_arg(dim, mode)
        
        solver = hpipm.hpipm_ocp_qp_solver(dim, arg)
        solver.solve(qp, qp_sol)

        status = solver.get('status')

        
        if status == 0:  
            u0 = qp_sol.get('u', 0)
            u0 = np.asarray(u0).flatten()
            rospy.logdebug(f"steer : {u0[1]}, accel : {u0[0]}")
            accel_cmd = float(u0[0])
            steer_cmd = float(u0[1])
            
            if accel_cmd >= 0:
                accel = accel_cmd
                brake = 0.0
            else:
                accel = 0.0
                brake = -accel_cmd
                
            return steer_cmd, accel, brake
        else:
            if self.last_mpc_error is None:
                rospy.logwarn_throttle(1.0, "⚠️ HPIPM 실패(status=%s) -> 스탠리(Stanley) 시스템 대체", status)
            else:
                rospy.logwarn_throttle(
                    1.0,
                    "⚠️ HPIPM 실패(status=%s, e_ct=%.3f m, e_yaw=%.3f rad, v=%.3f m/s) -> 스탠리(Stanley) 시스템 대체",
                    status,
                    self.last_mpc_error['e_ct'],
                    self.last_mpc_error['e_yaw'],
                    self.last_mpc_error['v'],
                )
            steer, accel, brake = self.run_stanley_imported_fallback()
            return steer, accel, brake

    def limit_steering_rate(self, steer):
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))
        max_step = max(0.0, self.max_dsteer * self.dt)
        steer = float(np.clip(
            steer,
            self.prev_steer_cmd - max_step,
            self.prev_steer_cmd + max_step,
        ))
        self.prev_steer_cmd = steer
        return steer
        
    def get_state_space_matrices(self, v, phi, delta):
        A = np.zeros((self.nx, self.nx))
        B = np.zeros((self.nx, self.nu))
    
        v_eps = max(0.1 , v)

        A[0, 2] = math.cos(phi)
        A[0, 3] = -v_eps * math.sin(phi)
        A[1, 2] = math.sin(phi)
        A[1, 3] = v_eps * math.cos(phi)
        A[3, 2] = math.tan(delta) / self.wheelbase

        B[2, 0] = 1.0  
        B[3, 1] = v_eps / (self.wheelbase * (math.cos(delta)**2))  

        A_d = np.eye(self.nx) + A * self.dt
        B_d = B * self.dt

        return A_d, B_d
    
    def prepare_mpc_matrices(self, xbar, xref, dref):
        T = self.T 
        A_list, B_list = [], []
        Q_list, R_list, q_list, r_list = [], [], [], []

        for t in range(T):
            A_d, B_d = self.get_state_space_matrices(
                v=xbar[2, t], phi=xbar[3, t], delta=dref[0, t]
            )

            A_list.append(A_d)
            B_list.append(B_d)

            Q_list.append(self.Q)
            R_list.append(self.R)

            q_step = -1.0 * np.dot(self.Q, xref[:, t])
            r_step = np.zeros(self.nu) 
            
            q_list.append(q_step)
            r_list.append(r_step)

        Q_list.append(self.Qf)
        q_list.append(-1.0 * np.dot(self.Qf, xref[:, T]))

        return A_list, B_list, Q_list, R_list, q_list, r_list

    # def save_data(self):
    #     if not self.error_log:
    #         rospy.logwarn(f"현재 수집된 데이터가 0개입니다.")
    #         return

    #     rospy.loginfo(f"총 {len(self.error_log)}개의 주행 데이터를 엑셀(CSV) 파일로 내보내는 중...")
    #     try:
    #         os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
    #         with open(self.csv_path, 'w', newline='', encoding='utf-8') as f:
    #             writer = csv.writer(f)
    #             writer.writerow(['Time (s)', 'CrossTrack Error (m)', 'Velocity (m/s)'])
    #             writer.writerows(self.error_log)
    #         rospy.loginfo(f"🎉 [파일 저장 성공] 경로: {self.csv_path}")
    #     except Exception as e:
    #         rospy.logerr(f"파일 저장 중 실패: {e}")

    def run(self):
        start_time = rospy.get_time()
        
        while not rospy.is_shutdown():
            if not self.odom_received:
                rospy.logwarn_throttle(2.0, "odometry 수신 대기 중입니다...")
                self.rate.sleep()
                continue

            target_x, target_y, target_yaw = self.find_closest_waypoint()
            
            if target_x is None:
                rospy.logwarn_throttle(2.0, "글로벌 경로를 기다리는 중입니다...")
                self.rate.sleep()
                continue

            e_yaw = self.current_yaw - target_yaw
            e_yaw = math.atan2(math.sin(e_yaw), math.cos(e_yaw))

            dx = self.current_x - target_x
            dy = self.current_y - target_y
            e_ct = -dx * math.sin(target_yaw) + dy * math.cos(target_yaw)
            self.last_mpc_error = {
                'e_ct': e_ct,
                'e_yaw': e_yaw,
                'v': self.current_v,
            }

            sim_time = rospy.get_time() - start_time
            self.error_log.append([sim_time, e_ct, self.current_v])

            x0 = np.array([0.0, e_ct, self.current_v, e_yaw])

            xref = np.zeros((self.nx, self.T + 1))
            dref = np.zeros((1, self.T))
            
            for t in range(self.T + 1):
                xref[0, t] = t * self.target_v * self.dt  
                xref[1, t] = 0.0                          
                xref[2, t] = self.target_v                
                xref[3, t] = 0.0                          

            xbar = np.zeros((self.nx, self.T + 1))
            for t in range(self.T + 1):
                xbar[:, t] = xref[:, t]  

            A_list, B_list, Q_list, R_list, q_list, r_list = self.prepare_mpc_matrices(xbar, xref, dref)
            
            steer, accel, brake = self.solve_mpc(x0, A_list, B_list, Q_list, R_list, q_list, r_list)
            raw_steer = steer
            steer = self.limit_steering_rate(steer)
            rospy.logdebug(
                "MPC command raw_steer=%.3f limited_steer=%.3f accel=%.3f brake=%.3f e_ct=%.3f e_yaw=%.3f",
                raw_steer,
                steer,
                accel,
                brake,
                e_ct,
                e_yaw,
            )
            
            ctrl_msg = CtrlCmd()
            ctrl_msg.longlCmdType = 1 
            
            ctrl_msg.steering = steer
            # ctrl_msg.accel = accel
            # ctrl_msg.brake = brake
            
            ctrl_msg.accel = np.clip(accel / self.max_accel, 0.0, 1.0)
            ctrl_msg.brake = np.clip(brake / self.max_brake, 0.0, 1.0)
            
            self.ctrl_pub.publish(ctrl_msg)
            self.rate.sleep()

if __name__ == '__main__':
    controller = None
    try:
        controller = MPCController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    # finally:
    #     if controller is not None:
    #         controller.save_data()
