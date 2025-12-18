"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

"""

import numpy as np
import math

# sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
from angle import angle_mod

# Pure Pursuit / 제어 파라미터
k = 0.4  # look-forward gain, 차량 속도 변화에 따른 look-ahead 거리 보정
Lfc = 0.5  # [m] look-ahead 거리, 실제 ERP42 회전 반경 고려
Kp = 2.0  # 속도 비례 제어 게인, ERP42 속도 응답 고려
dt = 0.01  # [s] 제어 루프 주기 (10Hz 정도)
WB = 1.04  # [m] 휠베이스, ERP42 실제 기준

# 차량 물리 파라미터
LENGTH = 1.6  # 차량 길이
WIDTH = 1.16  # 차량 폭
WHEEL_LEN = 0.54  # 바퀴 길이
WHEEL_WIDTH = 0.175  # 바퀴 폭
MAX_STEER = 28 * math.pi / 180  # 최대 조향 각도 (28도, ERP42 실제)

show_animation = True
pause_simulation = False  # Flag for pause simulation
is_reverse_mode = False  # Flag for reverse driving mode


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, is_reverse=False):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direction = -1 if is_reverse else 1  # Direction based on reverse flag
        self.rear_x = self.x - self.direction * ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - self.direction * ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.direction * self.v / WB * math.tan(delta) * dt
        self.yaw = angle_mod(self.yaw)
        self.v += a * dt
        self.rear_x = self.x - self.direction * ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - self.direction * ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.direction = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.direction.append(state.direction)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind], self.cy[ind])
            while ind < len(self.cx) - 1:  # <-- len(cx)-1까지만 반복
                distance_next_index = state.calc_distance(
                    self.cx[ind + 1], self.cy[ind + 1]
                )
                if distance_this_index < distance_next_index:
                    break
                ind += 1
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind
        # print(self.old_nearest_point_index)

        def calc_lookahead_gain(v):
            # v: [m/s]
            if v < 2.0:  # 7.2 km/h 이하
                return 0.3
            elif v < 4.5:  # 18 km/h 이하
                return 0.4
            else:  # 20 km/h 이상
                return 0.5

        k = calc_lookahead_gain(state.v)

        Lf = k * state.v + Lfc  # update look ahead distance
        print(Lf)

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    # Reverse steering angle when reversing
    delta = state.direction * math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
    # print(state.direction)

    # Limit steering angle to max value
    delta = np.clip(delta, -MAX_STEER, MAX_STEER)

    return delta, ind
