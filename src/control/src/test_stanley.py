#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math

class state:
    """ 차량의 현재 물리 상태를 저장하는 구조체 클래스 """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

class stanley():
    """ 순수 기하학 기반 Stanley 제어 연산 알고리즘 클래스 """
    def __init__(self):
        self.L = 3.0      # 차량 휠베이스
        self.kv = 0.25      # 저속 댐핑 안정화 게인
    
    def norm_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def cal_target(self, state, cx, cy):
        fx = state.x + self.L * np.cos(state.yaw)
        fy = state.y + self.L * np.sin(state.yaw)

        dx = [fx - point_x for point_x in cx]
        dy = [fy - point_y for point_y in cy]

        d = np.hypot(dx, dy)
        target_ind = int(np.argmin(d))
        
        front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -np.sin(state.yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_ind], dy[target_ind]], front_axle_vec)

        return target_ind, error_front_axle

    def stan_control(self, state, cx, cy, cyaw, h_gain, c_gain):
        current_target_ind, error_front_axle = self.cal_target(state, cx, cy)
        theta_e = self.norm_angle(cyaw[current_target_ind] - state.yaw) * h_gain
        theta_d = np.arctan2(c_gain * error_front_axle, self.kv + state.v)

        delta = theta_d + theta_e

        if abs(error_front_axle) < 0.05 and abs(theta_e) < math.radians(1.0):
            delta = 0.0

        delta = np.clip(delta, math.radians(-40), math.radians(40))
        return delta