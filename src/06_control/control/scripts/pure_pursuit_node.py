#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pure_pursuit_node.py

Pure Pursuit lateral controller with dynamic speed limit based on MGeo link_id.

알고리즘:
  1. 차량 후륜 중심에서 lookahead 거리(L_d) 이상 떨어진 경로 상의 목표점 탐색
  2. 목표점 방향각 α 계산
  3. 조향각: δ = atan(2 * L * sin(α) / L_d)
     (L = wheelbase, α = 차량 진행 방향과 목표점 방향의 각도 차이)

속도 정책 (MGeo link_type 기반):
  link_type=6  (일반 도로) → 60 km/h
  link_type=1  (교차로)    → 40 km/h

속도 적응형 lookahead:
  L_d = k_ld * v + L_d_min   (속도가 빠를수록 더 멀리 봄)

Subscribes:
  /global_path                   (nav_msgs/Path)
  /localization/kinematic_state  (nav_msgs/Odometry)
  /morai/ego_vehicle_status      (morai_msgs/EgoVehicleStatus)

Publishes:
  /cmd                           (morai_msgs/CtrlCmd)
"""

import math
import json
import os
import numpy as np
import rospy
import rospkg
from nav_msgs.msg import Path, Odometry
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from tf.transformations import euler_from_quaternion


# ---------------------------------------------------------------------------
# Speed table (km/h → m/s)
# ---------------------------------------------------------------------------
KMH_TO_MS = 1.0 / 3.6

SPEED_STRAIGHT_KMH = 60.0
SPEED_CURVE_KMH    = 40.0

SPEED_STRAIGHT_MS  = SPEED_STRAIGHT_KMH * KMH_TO_MS
SPEED_CURVE_MS     = SPEED_CURVE_KMH    * KMH_TO_MS


def _get_target_speed(link: dict) -> float:
    """MGeo link dict → 목표속도 (m/s)"""
    lt = str(link.get('link_type', ''))
    return SPEED_STRAIGHT_MS if lt == '6' else SPEED_CURVE_MS


class PurePursuitNode:
    # ------------------------------------------------------------------
    # Vehicle geometry (Hyundai Ioniq 5)
    # ------------------------------------------------------------------
    WHEELBASE = 3.000   # m  (front axle to rear axle)

    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=False)

        # ---- Parameters -----------------------------------------------
        def get_p(name, default):
            return rospy.get_param(f'~pure_pursuit/{name}',
                                   rospy.get_param(f'~{name}', default))

        # Lookahead: L_d = k_ld * v + L_d_min
        # ↓ k_ld 낮추면 커브에서 빠른 반응 (늦게 꺾는 문제 해결)
        self.k_ld      = get_p('k_ld',      0.15)   # lookahead gain [m/(m/s)]
        self.L_d_min   = get_p('L_d_min',   2.5)    # 최소 lookahead [m]
        self.L_d_max   = get_p('L_d_max',   10.0)   # 최대 lookahead [m]

        # Speed control
        self.accel_value = get_p('accel_value', 0.4)

        # Steering limits
        self.max_steer_deg = get_p('max_steer_deg', 40.0)
        self.max_steer_rad = math.radians(self.max_steer_deg)

        # 속도 적응형 EMA 필터:
        #   alpha = alpha_low + (alpha_high - alpha_low) * exp(-speed / speed_ref)
        #   저속 → alpha_high (빠른 반응), 고속 → alpha_low (강한 필터링)
        self.steer_alpha_low   = get_p('steer_alpha_low',   0.30)  # 고속 (강한 필터)
        self.steer_alpha_high  = get_p('steer_alpha_high',  0.70)  # 저속 (빠른 반응)
        self.steer_alpha_speed = get_p('steer_alpha_speed', 8.0)   # 전환 기준 속도 [m/s]

        # Control frequency
        self.control_hz  = get_p('control_hz', 20.0)

        # Path index search window
        self.max_idx_jump = get_p('max_idx_jump', 10)

        # ---- MGeo link table ------------------------------------------
        default_link_file = os.path.join(
            rospkg.RosPack().get_path('hdmap_loader'), 'scripts', 'link_set.json'
        )
        link_file = rospy.get_param('~link_file', default_link_file)
        self.link_table: dict = {}
        self._load_link_table(link_file)

        # ---- State ----------------------------------------------------
        self.path        = None
        self.path_xy: np.ndarray = None   # (N, 2)
        self.odom        = None
        self.closest_idx = 0
        self.path_updated = False

        self.current_link_id: str = ''
        self.target_speed: float  = SPEED_STRAIGHT_MS

        self.prev_steer: float = 0.0   # EMA 상태

        # ---- Publishers -----------------------------------------------
        self.cmd_pub = rospy.Publisher('/cmd', CtrlCmd, queue_size=1)

        # ---- Subscribers ----------------------------------------------
        rospy.Subscriber('/global_path',
                         Path, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/kinematic_state',
                         Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/morai/ego_vehicle_status',
                         EgoVehicleStatus, self.ego_status_callback, queue_size=1)

        # ---- Control loop ---------------------------------------------
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_hz),
            self.control_loop
        )

        rospy.loginfo(
            f"[PurePursuit] Started | "
            f"k_ld={self.k_ld} L_d=[{self.L_d_min},{self.L_d_max}]m "
            f"straight={SPEED_STRAIGHT_KMH:.0f} km/h  "
            f"curve={SPEED_CURVE_KMH:.0f} km/h  "
            f"max_steer={self.max_steer_deg}°"
        )

    # ------------------------------------------------------------------
    # MGeo 로드
    # ------------------------------------------------------------------
    def _load_link_table(self, path: str):
        if not os.path.exists(path):
            rospy.logerr(f"[PurePursuit] link_set.json not found: {path}")
            return
        try:
            with open(path, 'r') as f:
                links = json.load(f)
            for lnk in links:
                idx = lnk.get('idx')
                if idx:
                    self.link_table[idx] = lnk
            rospy.loginfo(f"[PurePursuit] Loaded {len(self.link_table)} links.")
        except Exception as e:
            rospy.logerr(f"[PurePursuit] Failed to load link_set.json: {e}")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def path_callback(self, msg: Path):
        if not msg.poses:
            rospy.logwarn("[PurePursuit] Empty /global_path, ignoring.")
            return
        xs = np.array([p.pose.position.x for p in msg.poses])
        ys = np.array([p.pose.position.y for p in msg.poses])
        self.path_xy = np.stack([xs, ys], axis=1)
        self.path    = msg
        self.closest_idx  = 0
        self.path_updated = True
        rospy.loginfo(f"[PurePursuit] Path received: {len(msg.poses)} waypoints.")

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def ego_status_callback(self, msg: EgoVehicleStatus):
        new_id = msg.link_id
        if new_id == self.current_link_id:
            return
        self.current_link_id = new_id
        link = self.link_table.get(new_id)
        if link:
            self.target_speed = _get_target_speed(link)
            rospy.loginfo(
                f"[PurePursuit] link={new_id} "
                f"type={link.get('link_type')} signal={link.get('related_signal')} "
                f"→ {self.target_speed * 3.6:.1f} km/h"
            )
        else:
            self.target_speed = SPEED_CURVE_MS
            rospy.logwarn(f"[PurePursuit] Unknown link_id={new_id}, fallback {SPEED_CURVE_KMH:.0f} km/h")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _normalize_angle(a: float) -> float:
        while a >  math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    def _get_pose(self):
        """(x, y, yaw, speed) from odometry."""
        p = self.odom.pose.pose
        t = self.odom.twist.twist
        q = p.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        speed = math.hypot(t.linear.x, t.linear.y)
        return p.position.x, p.position.y, yaw, speed

    def _find_closest_idx(self, rx: float, ry: float) -> int:
        """후륜 위치 기준 가장 가까운 waypoint index."""
        path = self.path_xy
        n = len(path)

        if self.path_updated:
            dists = np.hypot(path[:, 0] - rx, path[:, 1] - ry)
            self.path_updated = False
            return int(np.argmin(dists))

        start = self.closest_idx
        if start >= n:
            return n - 1
        end = min(start + max(self.max_idx_jump, 15), n)
        sub = path[start:end]
        dists = np.hypot(sub[:, 0] - rx, sub[:, 1] - ry)
        target_idx = start + int(np.argmin(dists))

        if target_idx - self.closest_idx > self.max_idx_jump:
            target_idx = self.closest_idx + self.max_idx_jump
        return target_idx

    # ------------------------------------------------------------------
    # Pure Pursuit
    # ------------------------------------------------------------------
    def _pure_pursuit(self, rx: float, ry: float,
                      yaw: float, speed: float) -> tuple:
        """
        Pure Pursuit 조향각 계산 (선형 보간으로 정밀한 목표점 선택).

        Returns:
            steer_raw (float): 조향각 [rad]
            idx       (int):   현재 closest_idx
            L_d       (float): 적용된 lookahead 거리 [m]
            goal_idx  (int):   선택된 lookahead waypoint index
        """
        path = self.path_xy
        n    = len(path)

        # 1. closest idx 갱신
        idx = self._find_closest_idx(rx, ry)
        self.closest_idx = idx

        # 2. 속도 적응형 lookahead distance
        L_d = float(np.clip(self.k_ld * speed + self.L_d_min,
                             self.L_d_min, self.L_d_max))

        # 3. Lookahead point 탐색 + 선형 보간
        #    세그먼트 [i-1, i] 위에서 정확히 L_d 거리인 점을 보간
        gx, gy = path[n - 1, 0], path[n - 1, 1]  # fallback: 마지막 점
        goal_idx = n - 1
        for i in range(idx + 1, n):
            dist = math.hypot(path[i, 0] - rx, path[i, 1] - ry)
            if dist >= L_d:
                # 이전 점과 현재 점 사이에서 선형 보간
                prev_dist = math.hypot(path[i - 1, 0] - rx, path[i - 1, 1] - ry)
                if dist > prev_dist:  # 단조 증가 보장
                    t = (L_d - prev_dist) / (dist - prev_dist)
                    gx = path[i - 1, 0] + t * (path[i, 0] - path[i - 1, 0])
                    gy = path[i - 1, 1] + t * (path[i, 1] - path[i - 1, 1])
                else:
                    gx, gy = path[i, 0], path[i, 1]
                goal_idx = i
                break

        # 4. 목표점 방향각 α  (차량 좌표계)
        dx  = gx - rx
        dy  = gy - ry
        angle_to_goal = math.atan2(dy, dx)
        alpha = self._normalize_angle(angle_to_goal - yaw)

        # 5. δ = atan(2 * L * sin(α) / L_d)
        steer_raw = math.atan2(2.0 * self.WHEELBASE * math.sin(alpha), L_d)
        steer_raw = float(np.clip(steer_raw, -self.max_steer_rad, self.max_steer_rad))

        return steer_raw, idx, L_d, goal_idx

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def control_loop(self, event):
        if self.path is None or self.path_xy is None or len(self.path_xy) == 0:
            rospy.logwarn_throttle(5.0, "[PurePursuit] Waiting for /global_path ...")
            return
        if self.odom is None:
            rospy.logwarn_throttle(5.0, "[PurePursuit] Waiting for /localization/kinematic_state ...")
            return

        rx, ry, yaw, speed = self._get_pose()

        n = len(self.path_xy)
        if self.closest_idx >= n - 5:
            rospy.loginfo_throttle(2.0, "[PurePursuit] Reached end of path. Stopping.")
            self._publish_stop()
            return

        # ── Pure Pursuit 계산 ──────────────────────────────────────────
        steer_raw, idx, L_d, goal_idx = self._pure_pursuit(rx, ry, yaw, speed)

        # ── 속도 적응형 EMA 필터 ────────────────────────────────────────
        # 저속: alpha_high (빠른 반응), 고속: alpha_low (강한 필터링)
        # alpha = low + (high - low) * exp(-speed / speed_ref)
        alpha = (self.steer_alpha_low
                 + (self.steer_alpha_high - self.steer_alpha_low)
                 * math.exp(-speed / max(self.steer_alpha_speed, 0.1)))
        steer_rad = alpha * steer_raw + (1.0 - alpha) * self.prev_steer
        steer_rad = float(np.clip(steer_rad, -self.max_steer_rad, self.max_steer_rad))
        self.prev_steer = steer_rad

        # ── 속도 제어 ──────────────────────────────────────────────────
        target      = self.target_speed
        speed_error = target - speed

        if speed_error > 0:
            accel = min(self.accel_value,
                        self.accel_value * (speed_error / max(target, 0.1) + 0.5))
            brake = 0.0
        else:
            accel = 0.0
            brake = min(0.6, abs(speed_error) * 0.1)

        # ── 발행 ───────────────────────────────────────────────────────
        cmd = CtrlCmd()
        cmd.ctrl_mode = 2   # AutoMode
        cmd.gear      = 4   # Drive
        cmd.cmd_type  = 1   # Throttle/Brake/Steer
        cmd.accel     = float(accel)
        cmd.brake     = float(brake)
        cmd.steer     = float(steer_rad)
        self.cmd_pub.publish(cmd)

        rospy.loginfo_throttle(
            0.5,
            f"[PurePursuit] idx={idx}/{n} goal={goal_idx} L_d={L_d:.1f}m "
            f"α_ema={alpha:.2f} "
            f"tgt={target*3.6:.1f}km/h cur={speed*3.6:.1f}km/h "
            f"steer={math.degrees(steer_rad):.1f}° "
            f"accel={accel:.2f} brake={brake:.2f}"
        )

    def _publish_stop(self):
        cmd = CtrlCmd()
        cmd.ctrl_mode = 2
        cmd.gear      = 4
        cmd.cmd_type  = 1
        cmd.accel     = 0.0
        cmd.brake     = 1.0
        cmd.steer     = 0.0
        self.cmd_pub.publish(cmd)


# -----------------------------------------------------------------------
if __name__ == '__main__':
    try:
        PurePursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
