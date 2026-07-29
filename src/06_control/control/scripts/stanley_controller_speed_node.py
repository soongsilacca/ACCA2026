#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
stanley_controller_speed_node.py

Stanley lateral controller with dynamic speed limit based on MGeo link_id.

link_set.json의 link_type / related_signal을 참조하여 목표 속도를 동적으로 결정:
  - 일반 도로 (link_type=6, related_signal=None) → 직선 → 60 km/h
  - 교차로 직진 (link_type=1, related_signal='straight') → 40 km/h
  - 교차로 좌회전 (link_type=1, related_signal='left'|'left_unprotected') → 40 km/h
  - 교차로 우회전 (link_type=1, related_signal='right_unprotected') → 40 km/h
  - 유턴 (related_signal='uturn_normal') → 40 km/h
  - 그 외 → 40 km/h (보수적)

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

SPEED_STRAIGHT_KMH  = 60.0   # 일반 직선 도로
SPEED_CURVE_KMH     = 40.0   # 교차로 직진 / 좌우회전 / 유턴

SPEED_STRAIGHT_MS   = SPEED_STRAIGHT_KMH * KMH_TO_MS
SPEED_CURVE_MS      = SPEED_CURVE_KMH    * KMH_TO_MS


def _is_straight_link(link: dict) -> bool:
    """
    MGeo 링크가 '직선 일반 도로'인지 판단.
      link_type=6  → 일반 도로 (교차로 아님)
      link_type=1  → 교차로 진입 구간 (신호 종류 불문)
    """
    lt = str(link.get('link_type', ''))
    return lt == '6'


def _get_target_speed(link: dict) -> float:
    """link 정보로 목표속도(m/s)를 반환."""
    if _is_straight_link(link):
        return SPEED_STRAIGHT_MS
    return SPEED_CURVE_MS


class StanleyControllerSpeed:
    # ------------------------------------------------------------------
    # Vehicle geometry (Hyundai Ioniq 5)
    # ------------------------------------------------------------------
    WHEELBASE      = 3.000   # m
    FRONT_OVERHANG = 0.845   # m
    REAR_OVERHANG  = 0.790   # m

    def __init__(self):
        rospy.init_node('stanley_controller_speed', anonymous=False)

        # ---- Parameters -----------------------------------------------
        def get_p(name, default):
            return rospy.get_param(f'~stanley/{name}', rospy.get_param(f'~{name}', default))

        self.k_e           = get_p('k_e',           0.8)   # CTE gain (낮게 시작, boost가 커버에서 보완)
        self.k_v           = get_p('k_v',           2.0)   # velocity softening (↑ 직선 oscillation 억제)
        self.accel_value   = get_p('accel_value',   0.4)
        self.brake_value   = get_p('brake_value',   0.0)
        self.lookahead_idx = get_p('lookahead_idx', 3)     # 짧게 유지 → 커브 진입 시 빠른 반응
        self.max_steer_deg = get_p('max_steer_deg', 40.0)
        self.control_hz    = get_p('control_hz',    20.0)
        self.max_idx_jump  = get_p('max_idx_jump',  10)

        # CTE deadband: |CTE| < deadband 이면 k_e를 0에 가깝게 줄임
        # → 직선 소진폭 oscillation 억제 (작은 CTE에서 과보정 방지)
        self.cte_deadband      = get_p('cte_deadband',      0.15)  # m
        self.cte_deadband_gain = get_p('cte_deadband_gain', 0.1)   # deadband 내 k_e 비율

        # 적응형 CTE gain boost: |CTE| > cte_boost_thresh 이면 k_e 증폭 (커브 복귀)
        self.cte_boost_thresh  = get_p('cte_boost_thresh',  0.6)   # m
        self.cte_boost_factor  = get_p('cte_boost_factor',  3.0)   # 배율

        # CTE 기반 자동 감속
        self.cte_speed_limit_thresh = get_p('cte_speed_limit_thresh', 1.5)  # m
        self.cte_speed_limit_ratio  = get_p('cte_speed_limit_ratio',  0.6)

        # ---- 모드 적응형 EMA 필터 (rate limiter 없음) ----------------------
        # heading_error_thresh 이상이면 커브 모드 → alpha_curve 사용 (빠른 반응)
        # 그 이하면 직선 모드 → alpha_straight 사용 (강한 평활화)
        # rate limiter는 위상 지연으로 oscillation을 악화시키므로 사용하지 않음
        self.heading_curve_thresh = math.radians(get_p('heading_curve_thresh_deg', 4.0))
        self.steer_alpha_straight = get_p('steer_alpha_straight', 0.25)  # 직선: 강한 필터
        self.steer_alpha_curve    = get_p('steer_alpha_curve',    0.7)   # 커브: 빠른 반응

        # link_set.json 경로 (hdmap_loader 패키지에서 읽음)
        default_link_file = os.path.join(
            rospkg.RosPack().get_path('hdmap_loader'), 'scripts', 'link_set.json'
        )
        link_file = rospy.get_param('~link_file', default_link_file)

        self.max_steer_rad = math.radians(self.max_steer_deg)

        # ---- Load MGeo link table ----------------------------------------
        self.link_table: dict = {}   # idx → link dict
        self._load_link_table(link_file)

        # ---- State -------------------------------------------------------
        self.path      = None
        self.odom      = None
        self.path_xy   = None        # (N, 2)
        self.closest_idx    = 0
        self.path_updated   = False

        # 현재 link_id (EgoVehicleStatus에서 갱신)
        self.current_link_id: str = ''
        self.target_speed: float  = SPEED_STRAIGHT_MS  # default: straight

        # 조향 필터 상태
        self.prev_steer: float = 0.0   # 직전 출력 조향각 [rad]

        # ---- Publishers --------------------------------------------------
        self.cmd_pub = rospy.Publisher('/cmd', CtrlCmd, queue_size=1)

        # ---- Subscribers -------------------------------------------------
        rospy.Subscriber('/global_path',
                         Path,
                         self.path_callback,
                         queue_size=1)
        rospy.Subscriber('/localization/kinematic_state',
                         Odometry,
                         self.odom_callback,
                         queue_size=1)
        rospy.Subscriber('/morai/ego_vehicle_status',
                         EgoVehicleStatus,
                         self.ego_status_callback,
                         queue_size=1)

        # ---- Control loop timer ------------------------------------------
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_hz),
            self.control_loop
        )

        rospy.loginfo(
            f"[Stanley-Speed] Node started | k_e={self.k_e} k_v={self.k_v} "
            f"straight={SPEED_STRAIGHT_KMH:.0f} km/h  curve={SPEED_CURVE_KMH:.0f} km/h  "
            f"max_steer={self.max_steer_deg}°"
        )

    # ------------------------------------------------------------------
    # MGeo link table 로드
    # ------------------------------------------------------------------
    def _load_link_table(self, path: str):
        if not os.path.exists(path):
            rospy.logerr(f"[Stanley-Speed] link_set.json not found: {path}")
            return
        try:
            with open(path, 'r') as f:
                links = json.load(f)
            for lnk in links:
                idx = lnk.get('idx')
                if idx:
                    self.link_table[idx] = lnk
            rospy.loginfo(f"[Stanley-Speed] Loaded {len(self.link_table)} links from {path}")
        except Exception as e:
            rospy.logerr(f"[Stanley-Speed] Failed to load link_set.json: {e}")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def path_callback(self, msg: Path):
        if len(msg.poses) == 0:
            rospy.logwarn("[Stanley-Speed] Received empty /global_path, ignoring.")
            return
        xs = np.array([p.pose.position.x for p in msg.poses])
        ys = np.array([p.pose.position.y for p in msg.poses])
        self.path_xy = np.stack([xs, ys], axis=1)
        self.path = msg
        self.closest_idx = 0
        self.path_updated = True
        rospy.loginfo(f"[Stanley-Speed] Path received: {len(msg.poses)} waypoints.")

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def ego_status_callback(self, msg: EgoVehicleStatus):
        new_link_id = msg.link_id
        if new_link_id != self.current_link_id:
            self.current_link_id = new_link_id
            link = self.link_table.get(new_link_id)
            if link:
                self.target_speed = _get_target_speed(link)
                signal = link.get('related_signal', 'N/A')
                lt     = link.get('link_type', 'N/A')
                rospy.loginfo(
                    f"[Stanley-Speed] link_id={new_link_id} "
                    f"type={lt} signal={signal} "
                    f"→ target_speed={self.target_speed*3.6:.1f} km/h"
                )
            else:
                # 알 수 없는 link_id → 보수적 속도
                self.target_speed = SPEED_CURVE_MS
                rospy.logwarn(
                    f"[Stanley-Speed] Unknown link_id={new_link_id}, "
                    f"fallback to {SPEED_CURVE_KMH:.0f} km/h"
                )

    # ------------------------------------------------------------------
    # Utility helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        while angle >  math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def _get_pose(self):
        """Extract (x, y, yaw, speed) from current odometry."""
        pose  = self.odom.pose.pose
        twist = self.odom.twist.twist
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        speed = math.hypot(twist.linear.x, twist.linear.y)
        return x, y, yaw, speed

    def _find_closest_idx(self, fx: float, fy: float) -> int:
        path = self.path_xy
        if path is None or len(path) == 0:
            return 0
        n = len(path)

        if self.path_updated:
            dists = np.hypot(path[:, 0] - fx, path[:, 1] - fy)
            closest = int(np.argmin(dists))
            self.path_updated = False
            return closest

        start = self.closest_idx
        if start >= n:
            return n - 1

        search_window = max(self.max_idx_jump, 15)
        end = min(start + search_window, n)

        sub = path[start:end]
        dists = np.hypot(sub[:, 0] - fx, sub[:, 1] - fy)
        local_idx = int(np.argmin(dists))
        target_idx = start + local_idx

        if target_idx - self.closest_idx > self.max_idx_jump:
            target_idx = self.closest_idx + self.max_idx_jump

        return target_idx

    # ------------------------------------------------------------------
    # Stanley steering computation
    # ------------------------------------------------------------------
    def _stanley_steer(self, x: float, y: float, yaw: float, speed: float):
        """
        적응형 Stanley 조향:
          δ = θ_e + arctan(k_e_eff * e / (k_v + v))

          k_e_eff 결정 규칙:
            |CTE| < deadband          → k_e * deadband_gain  (소진폭 보정 최소화)
            deadband ≤ |CTE| < boost  → k_e                  (기본 gain)
            |CTE| ≥ boost             → k_e * boost_factor    (커브/큰 오차 복귀)
        """
        path = self.path_xy
        if path is None or len(path) == 0:
            return 0.0, 0, 0.0, 0.0
        n = len(path)

        fx = x + self.WHEELBASE * math.cos(yaw)
        fy = y + self.WHEELBASE * math.sin(yaw)

        idx = self._find_closest_idx(fx, fy)
        self.closest_idx = idx

        heading_idx = min(idx + self.lookahead_idx, n - 1)
        if heading_idx > idx:
            dx = path[heading_idx, 0] - path[idx, 0]
            dy = path[heading_idx, 1] - path[idx, 1]
        else:
            prev_idx = max(idx - 1, 0)
            dx = path[idx, 0] - path[prev_idx, 0]
            dy = path[idx, 1] - path[prev_idx, 1]

        path_yaw = math.atan2(dy, dx)
        heading_error = self._normalize_angle(path_yaw - yaw)

        nearest_x = path[idx, 0]
        nearest_y = path[idx, 1]
        ex = nearest_x - fx
        ey = nearest_y - fy
        # CTE: 양수 → 차량이 경로 우측
        cte = math.cos(path_yaw) * ey - math.sin(path_yaw) * ex

        # ── 3단계 적응형 k_e ──────────────────────────────────────────
        abs_cte = abs(cte)
        if abs_cte < self.cte_deadband:
            # deadband: 거의 경로 위 → 과보정 방지
            k_e_eff = self.k_e * self.cte_deadband_gain
        elif abs_cte >= self.cte_boost_thresh:
            # 커브/큰 이탈: 선형 증폭
            excess_ratio = min(
                (abs_cte - self.cte_boost_thresh) / self.cte_boost_thresh, 1.0
            )
            factor = 1.0 + excess_ratio * (self.cte_boost_factor - 1.0)
            k_e_eff = self.k_e * factor
        else:
            k_e_eff = self.k_e

        cte_term = math.atan2(k_e_eff * cte, self.k_v + speed)
        steer_raw = heading_error + cte_term
        steer_raw = max(-self.max_steer_rad, min(self.max_steer_rad, steer_raw))
        return steer_raw, idx, cte, heading_error

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def control_loop(self, event):
        if self.path is None or self.path_xy is None or len(self.path_xy) == 0:
            rospy.logwarn_throttle(5.0, "[Stanley-Speed] Waiting for /global_path ...")
            return
        if self.odom is None:
            rospy.logwarn_throttle(5.0, "[Stanley-Speed] Waiting for /localization/kinematic_state ...")
            return

        x, y, yaw, speed = self._get_pose()

        n = len(self.path_xy)
        if self.closest_idx >= n - 5:
            rospy.loginfo_throttle(2.0, "[Stanley-Speed] Reached end of path. Sending stop command.")
            self._publish_stop()
            return

        steer_raw, idx, cte, heading_err = self._stanley_steer(x, y, yaw, speed)

        # ── 모드 적응형 EMA (rate limiter 없음) ──────────────────────────
        # heading_error가 크면 커브 모드 → alpha_curve (빠른 반응)
        # 작으면 직선 모드 → alpha_straight (강한 평활화, oscillation 억제)
        if abs(heading_err) > self.heading_curve_thresh:
            alpha = self.steer_alpha_curve      # 커브: 빠르게 반응
            mode  = 'CURVE'
        else:
            alpha = self.steer_alpha_straight   # 직선: 강하게 필터링
            mode  = 'STRAIGHT'

        steer_rad = alpha * steer_raw + (1.0 - alpha) * self.prev_steer
        steer_rad = max(-self.max_steer_rad, min(self.max_steer_rad, steer_rad))
        self.prev_steer = steer_rad

        # --- 동적 속도 제어 (CTE 기반 자동 감속 포함) ---
        target = self.target_speed

        # CTE가 크면 목표속도를 제한하여 조향 여유를 확보
        if abs(cte) > self.cte_speed_limit_thresh:
            target = min(target, target * self.cte_speed_limit_ratio)
            rospy.logwarn_throttle(
                1.0,
                f"[Stanley-Speed] Large CTE={cte:.2f}m → speed capped to {target*3.6:.1f} km/h"
            )

        speed_error = target - speed

        if speed_error > 0:
            accel = min(self.accel_value,
                        self.accel_value * (speed_error / max(target, 0.1) + 0.5))
            brake = 0.0
        else:
            accel = 0.0
            brake = min(0.6, abs(speed_error) * 0.1)

        cmd = CtrlCmd()
        cmd.ctrl_mode = 2
        cmd.gear      = 4
        cmd.cmd_type  = 1
        cmd.accel     = float(accel)
        cmd.brake     = float(brake)
        cmd.steer     = float(steer_rad)

        self.cmd_pub.publish(cmd)

        rospy.loginfo_throttle(
            0.5,
            f"[Stanley-Speed][{mode}] idx={idx}/{n} CTE={cte:.2f}m he={math.degrees(heading_err):.1f}° "
            f"tgt={target*3.6:.1f}km/h cur={speed*3.6:.1f}km/h "
            f"steer={math.degrees(steer_rad):.1f}° accel={accel:.2f} brake={brake:.2f}"
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
        StanleyControllerSpeed()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
