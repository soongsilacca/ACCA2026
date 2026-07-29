#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
stanley_controller_node.py

Stanley lateral controller for ROS1.

Subscribes:
  /global_path          (nav_msgs/Path)         — reference path in 'map' frame
  /localization/kinematic_state  (nav_msgs/Odometry)  — vehicle pose + twist

Publishes:
  /cmd                  (morai_msgs/CtrlCmd)    — steering + throttle/brake

Vehicle: 2023 Hyundai Ioniq 5
  wheelbase       = 3.000 m
  front_overhang  = 0.845 m
  rear_overhang   = 0.790 m

Stanley method:
  delta = heading_error + atan2(k_e * cross_track_error, k_v + speed)

  The front-axle position is used for CTE computation:
    front_x = x + (wheelbase) * cos(yaw)
    front_y = y + (wheelbase) * sin(yaw)
  (rear-axle origin assumed to be the odometry frame origin, i.e. base_link ≈ rear axle midpoint)
"""

import math
import numpy as np
import rospy
from nav_msgs.msg import Path, Odometry
from morai_msgs.msg import CtrlCmd
from tf.transformations import euler_from_quaternion
from global_path_planner.msg import PlannerTrajectory


class StanleyController:
    # ------------------------------------------------------------------
    # Vehicle geometry (Hyundai Ioniq 5)
    # ------------------------------------------------------------------
    WHEELBASE      = 3.000   # m
    FRONT_OVERHANG = 0.845   # m
    REAR_OVERHANG  = 0.790   # m

    def __init__(self):
        rospy.init_node('stanley_controller', anonymous=False)

        # ---- Parameters -----------------------------------------------
        def get_p(name, default):
            return rospy.get_param(f'~stanley/{name}', rospy.get_param(f'~{name}', default))

        self.k_e           = get_p('k_e',           0.5)
        self.k_v           = get_p('k_v',           1.0)
        self.target_speed  = get_p('target_speed',  8.0)   # m/s
        self.accel_value   = get_p('accel_value',   0.4)
        self.brake_value   = get_p('brake_value',   0.0)
        self.lookahead_idx = get_p('lookahead_idx', 3)
        self.max_steer_deg = get_p('max_steer_deg', 40.0)
        self.control_hz    = get_p('control_hz',    20.0)

        self.max_steer_rad = math.radians(self.max_steer_deg)

        # ---- State -------------------------------------------------------
        self.path:     object   = None
        self.odom:     Odometry = None
        self.path_xy:  np.ndarray = None   # (N, 2) array of [x, y]
        self.target_speeds: np.ndarray = None
        self.closest_idx: int = 0
        self.path_updated: bool = False
        self.max_idx_jump: int = get_p('max_idx_jump', 10)

        # ---- Publishers --------------------------------------------------
        self.cmd_pub = rospy.Publisher('/cmd', CtrlCmd, queue_size=1)

        # ---- Subscribers -------------------------------------------------
        rospy.Subscriber('/local_trajectory',
                         PlannerTrajectory,
                         self.trajectory_callback,
                         queue_size=1)
        rospy.Subscriber('/global_path',
                         Path,
                         self.path_callback,
                         queue_size=1)
        rospy.Subscriber('/localization/kinematic_state',
                         Odometry,
                         self.odom_callback,
                         queue_size=1)

        # ---- Control loop timer ------------------------------------------
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_hz),
            self.control_loop
        )

        rospy.loginfo(
            f"[Stanley] Node started | k_e={self.k_e} k_v={self.k_v} "
            f"target_speed={self.target_speed} m/s max_steer={self.max_steer_deg}°"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def trajectory_callback(self, msg: PlannerTrajectory):
        if len(msg.waypoints) == 0:
            return
        xs = np.array([wp.position.x for wp in msg.waypoints])
        ys = np.array([wp.position.y for wp in msg.waypoints])
        speeds = np.array([wp.target_speed for wp in msg.waypoints])
        self.path_xy = np.stack([xs, ys], axis=1)
        self.target_speeds = speeds
        self.path = msg
        self.closest_idx = 0
        self.path_updated = True

    def path_callback(self, msg: Path):
        if self.path_xy is not None:
            return  # Prefer local_trajectory if available
        if len(msg.poses) == 0:
            return
        xs = np.array([p.pose.position.x for p in msg.poses])
        ys = np.array([p.pose.position.y for p in msg.poses])
        self.path_xy = np.stack([xs, ys], axis=1)
        self.path = msg
        self.closest_idx = 0
        self.path_updated = True

    def odom_callback(self, msg: Odometry):
        self.odom = msg

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
        pose = self.odom.pose.pose
        twist = self.odom.twist.twist

        x = pose.position.x
        y = pose.position.y

        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        speed = math.hypot(twist.linear.x, twist.linear.y)
        return x, y, yaw, speed

    def _find_closest_idx(self, fx: float, fy: float) -> int:
        """
        Find the closest waypoint index to the front axle position.
        If path was updated, perform a full global path search.
        Otherwise, search within a small forward window to prevent index jumping.
        """
        path = self.path_xy
        if path is None or len(path) == 0:
            return 0
        n = len(path)

        # 1. 경로가 새로 수신/업데이트된 경우: 전체 경로에서 차량 위치와 가장 가까운 idx 탐색
        if self.path_updated:
            dists = np.hypot(path[:, 0] - fx, path[:, 1] - fy)
            closest = int(np.argmin(dists))
            self.path_updated = False
            return closest

        # 2. 일반 추종 중: idx가 껑충 뛰지 않도록 이전 closest_idx 기준 좁은 영역만 탐색
        start = self.closest_idx
        if start >= n:
            return n - 1

        # 한 제어 주기 동안 이동할 수 있는 최대 웨이포인트 범위로 탐색 윈도우 제한
        search_window = max(self.max_idx_jump, 15)
        end = min(start + search_window, n)

        sub = path[start:end]
        dists = np.hypot(sub[:, 0] - fx, sub[:, 1] - fy)
        local_idx = int(np.argmin(dists))
        target_idx = start + local_idx

        # 껑충 뛰는 것(index jump) 최종 제한
        if target_idx - self.closest_idx > self.max_idx_jump:
            target_idx = self.closest_idx + self.max_idx_jump

        return target_idx

    # ------------------------------------------------------------------
    # Stanley steering computation
    # ------------------------------------------------------------------
    def _stanley_steer(self, x: float, y: float, yaw: float, speed: float) -> float:
        """
        Compute Stanley steering angle [rad].

        Front-axle position:
            fx = x + wheelbase * cos(yaw)
            fy = y + wheelbase * sin(yaw)
        """
        path = self.path_xy
        if path is None or len(path) == 0:
            return 0.0, 0
        n = len(path)

        # --- Front-axle position ---
        fx = x + self.WHEELBASE * math.cos(yaw)
        fy = y + self.WHEELBASE * math.sin(yaw)

        # --- Closest waypoint (search forward only) ---
        idx = self._find_closest_idx(fx, fy)
        self.closest_idx = idx

        # --- Path heading at closest point ---
        heading_idx = min(idx + self.lookahead_idx, n - 1)
        if heading_idx > idx:
            dx = path[heading_idx, 0] - path[idx, 0]
            dy = path[heading_idx, 1] - path[idx, 1]
        else:
            # Last segment: use previous direction
            prev_idx = max(idx - 1, 0)
            dx = path[idx, 0] - path[prev_idx, 0]
            dy = path[idx, 1] - path[prev_idx, 1]

        path_yaw = math.atan2(dy, dx)

        # --- Heading error ---
        heading_error = self._normalize_angle(path_yaw - yaw)

        # --- Cross-track error (signed, positive = path is to the left) ---
        # Vector from front-axle to nearest waypoint
        nearest_x = path[idx, 0]
        nearest_y = path[idx, 1]

        # CTE = cross product of path direction × (front-axle - nearest)
        # Sign convention: positive when vehicle is to the right of the path
        ex = nearest_x - fx
        ey = nearest_y - fy

        # Project onto path normal (rotate path heading 90° left)
        # sign: positive = vehicle is to the right → steer left
        cte = math.cos(path_yaw) * ey - math.sin(path_yaw) * ex

        # --- Stanley formula ---
        cte_term = math.atan2(self.k_e * cte, self.k_v + speed)
        steer = heading_error + cte_term

        # --- Clamp ---
        steer = max(-self.max_steer_rad, min(self.max_steer_rad, steer))
        return steer, idx

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def control_loop(self, event):
        # Guard: wait until both path and odom are available
        if self.path is None or self.path_xy is None or len(self.path_xy) == 0:
            rospy.logwarn_throttle(5.0, "[Stanley] Waiting for /global_path ...")
            return
        if self.odom is None:
            rospy.logwarn_throttle(5.0, "[Stanley] Waiting for /localization/kinematic_state ...")
            return

        x, y, yaw, speed = self._get_pose()

        # --- Check if we've reached the end of the path ---
        n = len(self.path_xy)
        if self.closest_idx >= n - 5:
            rospy.loginfo_throttle(2.0, "[Stanley] Reached end of path. Sending stop command.")
            self._publish_stop()
            return

        # --- Compute steering ---
        steer_rad, idx = self._stanley_steer(x, y, yaw, speed)
        # MORAI expects steer in radians, positive = turn left
        # (same sign convention as math.atan2)

        # --- Speed control (simple P-controller or constant) ---
        speed_error = self.target_speed - speed
        if speed_error > 0:
            accel = min(self.accel_value, self.accel_value * (speed_error / self.target_speed + 0.5))
            brake = 0.0
        else:
            accel = 0.0
            brake = min(0.6, abs(speed_error) * 0.1)

        # --- Build and publish CtrlCmd ---
        cmd = CtrlCmd()
        cmd.ctrl_mode = 2        # AutoMode
        cmd.gear      = 4        # Drive
        cmd.cmd_type  = 1        # Throttle/Brake/Steer mode
        cmd.accel     = float(accel)
        cmd.brake     = float(brake)
        cmd.steer     = float(steer_rad)

        self.cmd_pub.publish(cmd)

        rospy.logdebug(
            f"[Stanley] idx={idx}/{n} "
            f"speed={speed:.2f} steer={math.degrees(steer_rad):.1f}° "
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
        StanleyController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
