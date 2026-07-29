#!/usr/bin/env python3
"""
Autoware OpenPlanner & Universe Local Path Planner Node
=========================================================
Architecture:
- Quintic Frenet Lattice Rollout Generation (Multi-Lane Support up to ±6.5m span)
- 3-Circle Ego Vehicle Envelope Model for Obstacle Clearance Evaluation
- Continuous Cost Minimization (FSM-Free & Smooth Lane Avoidance)
- Curvature-Based Dynamic Speed Profiling & Distance-Based In-Path Deceleration
- Single-Source-of-Truth Trajectory & RViz Visualization Synchronization

Topics:
- Subscribed: /global_path, /localization/kinematic_state, /clusters_markers
- Published: /local_trajectory, /rollout_markers, /speed_markers, /planner_status
"""

import math
import numpy as np
from scipy.interpolate import CubicSpline

import rospy
import tf
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray

from global_path_planner.msg import PlannerTrajectory, PlannerWaypoint

KMH_TO_MS = 1.0 / 3.6
MS_TO_KMH = 3.6


class QuinticPolynomial:
    """5th-Order Frenet Spline Polynomial for Smooth Lateral Offset Transitions."""

    def __init__(self, xs: float, vxs: float, axs: float, xe: float, vxe: float, axe: float, T: float):
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0
        T = max(1.0, float(T))
        A = np.array([
            [T ** 3, T ** 4, T ** 5],
            [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
            [6 * T, 12 * T ** 2, 20 * T ** 3]
        ])
        b = np.array([
            xe - self.a0 - self.a1 * T - self.a2 * T ** 2,
            vxe - self.a1 - 2 * self.a2 * T,
            axe - 2 * self.a2
        ])
        try:
            x = np.linalg.solve(A, b)
            self.a3, self.a4, self.a5 = x
        except np.linalg.LinAlgError:
            self.a3 = self.a4 = self.a5 = 0.0

    def calc(self, t: float) -> float:
        return (self.a0 + self.a1 * t + self.a2 * t ** 2 + self.a3 * t ** 3 +
                self.a4 * t ** 4 + self.a5 * t ** 5)


class LocalPathPlannerNode:
    def __init__(self):
        rospy.init_node('local_path_planner_node', anonymous=False)

        # 1. ROS Parameters
        self.horizon_dist = rospy.get_param('~planning/horizon_dist', 50.0)
        self.step_size = rospy.get_param('~planning/step_size', 0.5)
        self.max_speed_kmh = rospy.get_param('~planning/max_speed_kmh', 60.0)
        self.min_speed_kmh = rospy.get_param('~planning/min_speed_kmh', 15.0)
        self.max_lat_accel = rospy.get_param('~planning/max_lat_accel', 2.0)
        self.max_drivable_offset = rospy.get_param('~planning/max_drivable_offset', 6.5)
        self.max_offset_rate = rospy.get_param('~planning/max_offset_rate', 0.20)
        self.publish_rate = rospy.get_param('~planning/publish_rate', 10.0)
        self.frame_id = rospy.get_param('~planning/frame_id', 'map')

        self.num_rollouts = rospy.get_param('~rollouts/num_rollouts', 41)
        self.rollout_spacing = rospy.get_param('~rollouts/rollout_spacing', 0.35)
        self.safety_margin = rospy.get_param('~rollouts/safety_margin', 1.2)
        self.enable_avoidance = rospy.get_param('~rollouts/enable_avoidance', True)
        self.min_stop_dist = rospy.get_param('~acc/min_stop_dist', 4.0)
        self.max_decel = rospy.get_param('~acc/max_decel', 2.5)

        # Vehicle Dimensions
        self.wheelbase = 3.0
        self.vehicle_width = 1.89
        self.vehicle_length = 4.635

        # Multi-Lane Rollouts (Clamped strictly within max_drivable_offset)
        half = self.num_rollouts // 2
        raw_offsets = np.linspace(-half * self.rollout_spacing, half * self.rollout_spacing, self.num_rollouts)
        self.offsets = np.clip(raw_offsets, -self.max_drivable_offset, self.max_drivable_offset)
        self.offsets = np.unique(self.offsets)

        # State Variables
        self.global_path = None
        self.global_xy = None
        self.odom = None
        self.obstacle_list = []
        self.filtered_offset = 0.0
        self.target_offset = 0.0
        self.max_speed_ms = self.max_speed_kmh * KMH_TO_MS

        # Publishers & Subscribers
        self.traj_pub = rospy.Publisher('/local_trajectory', PlannerTrajectory, queue_size=1)
        self.rollout_pub = rospy.Publisher('/rollout_markers', MarkerArray, queue_size=1)
        self.speed_pub = rospy.Publisher('/speed_markers', MarkerArray, queue_size=1)
        self.status_pub = rospy.Publisher('/planner_status', String, queue_size=1)

        self.tf_listener = tf.TransformListener()

        pose_topic = rospy.get_param('~pose_topic', '/localization/kinematic_state')
        obstacles_topic = rospy.get_param('~obstacles_topic', '/clusters_markers')
        rospy.Subscriber('/global_path', Path, self._cb_global_path, queue_size=1)
        rospy.Subscriber(pose_topic, Odometry, self._cb_odom, queue_size=1)
        rospy.Subscriber(obstacles_topic, MarkerArray, self._cb_clusters, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._planning_loop)
        rospy.loginfo('[LocalPathPlanner] Multi-Lane Planner ready (%d rollouts, ±%.2fm offset)',
                       len(self.offsets), self.max_drivable_offset)

    def _cb_global_path(self, msg: Path):
        if msg.poses:
            self.global_path = msg
            self.global_xy = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])

    def _cb_odom(self, msg: Odometry):
        self.odom = msg
        self._planning_loop(None)

    def _cb_clusters(self, msg: MarkerArray):
        obs = []
        for m in msg.markers:
            if m.action in (Marker.DELETE, Marker.DELETEALL):
                continue
            r = max(0.4, min(max(m.scale.x, m.scale.y) / 2.0, 2.5))
            frame_id = m.header.frame_id if m.header.frame_id else 'velodyne'
            ox, oy = m.pose.position.x, m.pose.position.y

            if frame_id != self.frame_id:
                transformed = False
                try:
                    p_stamped = PointStamped()
                    p_stamped.header = m.header
                    p_stamped.point = m.pose.position
                    p_trans = self.tf_listener.transformPoint(self.frame_id, p_stamped)
                    ox, oy = p_trans.point.x, p_trans.point.y
                    transformed = True
                except Exception:
                    pass

                if not transformed and self.odom:
                    p = self.odom.pose.pose
                    q = p.orientation
                    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                    lx = m.pose.position.x + (3.85 if frame_id == 'velodyne' else 0.0)
                    ly = m.pose.position.y
                    ox = p.position.x + lx * math.cos(yaw) - ly * math.sin(yaw)
                    oy = p.position.y + lx * math.sin(yaw) + ly * math.cos(yaw)

            obs.append((ox, oy, r))
        self.obstacle_list = obs

    def _closest_idx(self, ego_x, ego_y, ego_yaw):
        if self.global_xy is None:
            return 0
        dists = np.hypot(self.global_xy[:, 0] - ego_x, self.global_xy[:, 1] - ego_y)
        min_idx = int(np.argmin(dists))
        for idx in range(min_idx, min(min_idx + 10, len(self.global_xy) - 1)):
            dx = self.global_xy[idx, 0] - ego_x
            dy = self.global_xy[idx, 1] - ego_y
            heading = math.atan2(dy, dx)
            if abs(math.atan2(math.sin(heading - ego_yaw), math.cos(heading - ego_yaw))) < math.pi / 2.0:
                return idx
        return min_idx

    def _planning_loop(self, _):
        if self.global_xy is None or self.odom is None:
            return

        p = self.odom.pose.pose
        q = p.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        ego_x, ego_y = p.position.x, p.position.y

        start_idx = self._closest_idx(ego_x, ego_y, yaw)
        n_pts = len(self.global_xy)
        if start_idx >= n_pts - 5:
            return

        accum = 0.0
        end_idx = start_idx
        while end_idx < n_pts - 1 and accum < self.horizon_dist:
            accum += math.hypot(self.global_xy[end_idx + 1, 0] - self.global_xy[end_idx, 0],
                                self.global_xy[end_idx + 1, 1] - self.global_xy[end_idx, 1])
            end_idx += 1
        sub_pts = self.global_xy[start_idx:end_idx + 1]
        if len(sub_pts) < 3:
            return

        s_arr = np.zeros(len(sub_pts))
        for i in range(1, len(sub_pts)):
            s_arr[i] = s_arr[i - 1] + math.hypot(sub_pts[i, 0] - sub_pts[i - 1, 0],
                                                  sub_pts[i, 1] - sub_pts[i - 1, 1])
        _, uniq = np.unique(s_arr, return_index=True)
        sub_pts, s_arr = sub_pts[uniq], s_arr[uniq]

        cs_x = CubicSpline(s_arr, sub_pts[:, 0])
        cs_y = CubicSpline(s_arr, sub_pts[:, 1])
        s_dense = np.arange(0.0, s_arr[-1], self.step_size)
        x_base, y_base = cs_x(s_dense), cs_y(s_dense)
        yaw_base = np.arctan2(cs_y(s_dense, 1), cs_x(s_dense, 1))
        base_xy = np.stack([x_base, y_base], axis=1)

        # Frenet ego offset
        dx0, dy0 = ego_x - base_xy[0, 0], ego_y - base_xy[0, 1]
        d_ego = float(np.clip(-dx0 * math.sin(yaw_base[0]) + dy0 * math.cos(yaw_base[0]),
                              -self.max_drivable_offset, self.max_drivable_offset))

        # Filter road obstacles
        road_obstacles = []
        for ox, oy, r in self.obstacle_list:
            dists = np.hypot(base_xy[:, 0] - ox, base_xy[:, 1] - oy)
            idx = int(np.argmin(dists))
            lat = -(ox - base_xy[idx, 0]) * math.sin(yaw_base[idx]) + (oy - base_xy[idx, 1]) * math.cos(yaw_base[idx])
            if abs(lat) - r <= (self.max_drivable_offset + 0.6):
                road_obstacles.append((ox, oy, r))

        # Dynamic transition length
        closest_obs_s = 1e6
        for ox, oy, r in road_obstacles:
            dists = np.hypot(base_xy[:, 0] - ox, base_xy[:, 1] - oy)
            idx = int(np.argmin(dists))
            if dists[idx] - r < 3.5:
                closest_obs_s = min(closest_obs_s, idx * self.step_size)

        if closest_obs_s < 1e5:
            trans_s = float(np.clip(closest_obs_s * 0.75, 6.0, 20.0))
        else:
            trans_s = float(np.clip(abs(d_ego) * 3.5 + 5.0, 5.0, 12.0))

        # Continuous Cost Evaluation over all Rollouts
        best_offset = 0.0
        best_cost = 1e9
        foot_offsets = [0.0, self.wheelbase * 0.45, self.wheelbase * 0.9]
        n_dense = len(base_xy)
        rollout_geometries = []
        any_valid = False

        for d in self.offsets:
            qp = QuinticPolynomial(d_ego, 0.0, 0.0, d, 0.0, 0.0, trans_s)
            rollout = np.zeros_like(base_xy)
            for i in range(n_dense):
                s = i * self.step_size
                lat = qp.calc(s) if s < trans_s else d
                rollout[i, 0] = base_xy[i, 0] - math.sin(yaw_base[i]) * lat
                rollout[i, 1] = base_xy[i, 1] + math.cos(yaw_base[i]) * lat
            rollout_geometries.append(rollout)

            collision = False
            min_obs_dist = 1e6
            if self.enable_avoidance and road_obstacles:
                for ox, oy, r in road_obstacles:
                    circles = [(rollout[:, 0] + off * np.cos(yaw_base), rollout[:, 1] + off * np.sin(yaw_base)) for off in foot_offsets]
                    d_min = min(np.min(np.hypot(cx - ox, cy - oy)) for cx, cy in circles) - r
                    min_obs_dist = min(min_obs_dist, d_min)
                    if d_min < self.safety_margin:
                        collision = True
                        break

            if collision:
                continue

            any_valid = True
            total_cost = (abs(d)**1.5)*8.0 + abs(d - self.target_offset)*1.5 + (((3.0 - min_obs_dist)**2)*18.0 if min_obs_dist < 3.0 else 0.0)
            if total_cost < best_cost:
                best_cost = total_cost
                best_offset = d

        if any_valid:
            self.target_offset = best_offset
            status_str = f"TRACKING (offset: {self.target_offset:+.2f}m)"
        else:
            self.target_offset = self.filtered_offset
            status_str = "EMERGENCY_STOP (All rollouts blocked)"

        # Rate Limiting on Lateral Offset
        step = float(np.clip(self.target_offset - self.filtered_offset, -self.max_offset_rate, self.max_offset_rate))
        self.filtered_offset = float(np.clip(self.filtered_offset + step, -self.max_drivable_offset, self.max_drivable_offset))

        # Generate Executed Trajectory
        qp_active = QuinticPolynomial(d_ego, 0.0, 0.0, self.filtered_offset, 0.0, 0.0, trans_s)
        selected_xy = np.zeros_like(base_xy)
        for i in range(n_dense):
            s = i * self.step_size
            lat = qp_active.calc(s) if s < trans_s else self.filtered_offset
            selected_xy[i, 0] = base_xy[i, 0] - math.sin(yaw_base[i]) * lat
            selected_xy[i, 1] = base_xy[i, 1] + math.cos(yaw_base[i]) * lat

        # Headings & Curvature
        sel_yaw = np.copy(yaw_base)
        for i in range(n_dense - 1):
            hdg = math.atan2(selected_xy[i + 1, 1] - selected_xy[i, 1], selected_xy[i + 1, 0] - selected_xy[i, 0])
            if abs(math.atan2(math.sin(hdg - yaw_base[i]), math.cos(hdg - yaw_base[i]))) < math.pi / 3.0:
                sel_yaw[i] = hdg
        sel_yaw[-1] = sel_yaw[-2]

        curvature = np.zeros(n_dense)
        for i in range(1, n_dense - 1):
            x1, y1 = selected_xy[i - 1]; x2, y2 = selected_xy[i]; x3, y3 = selected_xy[i + 1]
            area = 0.5 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
            abc = math.hypot(x2 - x1, y2 - y1) * math.hypot(x3 - x2, y3 - y2) * math.hypot(x1 - x3, y1 - y3)
            if abc > 1e-6:
                curvature[i] = (4.0 * area) / abc

        # Speed Profiling
        closest_path_obs = 1e6
        for ox, oy, r in road_obstacles:
            dists = np.hypot(selected_xy[:, 0] - ox, selected_xy[:, 1] - oy)
            idx = int(np.argmin(dists))
            if dists[idx] - r < (self.vehicle_width / 2.0) + 0.15:
                closest_path_obs = min(closest_path_obs, idx * self.step_size)

        target_speeds = np.zeros(n_dense)
        for i in range(n_dense):
            s = i * self.step_size
            kappa = curvature[i]
            v_curve = math.sqrt(self.max_lat_accel / kappa) if kappa > 1e-3 else self.max_speed_ms
            v = min(self.max_speed_ms, np.clip(v_curve, self.min_speed_kmh * KMH_TO_MS, self.max_speed_ms))

            if closest_path_obs < 35.0:
                stop_s = max(0.0, closest_path_obs - self.min_stop_dist)
                v = 0.0 if s >= stop_s else min(v, math.sqrt(max(0.0, 2.0 * self.max_decel * (stop_s - s))))
            if not any_valid and closest_path_obs < 5.0:
                v = 0.0
            target_speeds[i] = v

        # Publish Planner Messages
        stamp = rospy.Time.now()
        traj = PlannerTrajectory()
        traj.header.frame_id, traj.header.stamp = self.frame_id, stamp
        for i in range(n_dense):
            wp = PlannerWaypoint()
            wp.position = Point(x=float(selected_xy[i, 0]), y=float(selected_xy[i, 1]), z=0.0)
            wp.yaw, wp.target_speed, wp.curvature = float(sel_yaw[i]), float(target_speeds[i]), float(curvature[i])
            traj.waypoints.append(wp)
        self.traj_pub.publish(traj)
        self.status_pub.publish(status_str)

        # Publish RViz Markers
        self._publish_markers(rollout_geometries, selected_xy, sel_yaw, target_speeds, stamp)

    def _publish_markers(self, rollout_geometries, selected_xy, sel_yaw, target_speeds, stamp):
        rollout_array = MarkerArray()
        best_idx = int(np.argmin([abs(d - self.filtered_offset) for d in self.offsets]))

        for idx, rollout in enumerate(rollout_geometries):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.frame_id, stamp
            m.ns, m.id, m.type, m.action = 'rollouts', idx, Marker.LINE_STRIP, Marker.ADD
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=0.1) for p in rollout]
            m.scale.x = 0.15 if idx == best_idx else 0.04
            m.color.r, m.color.g, m.color.b, m.color.a = (0.0, 1.0, 0.0, 1.0) if idx == best_idx else (0.6, 0.6, 0.6, 0.3)
            rollout_array.markers.append(m)
        self.rollout_pub.publish(rollout_array)

        speed_array = MarkerArray()
        del_m = Marker(); del_m.action = Marker.DELETEALL
        speed_array.markers.append(del_m)

        interval = max(1, int(2.0 / self.step_size))
        m_id = 0
        for i in range(0, len(selected_xy), interval):
            sp_kmh = target_speeds[i] * MS_TO_KMH
            yaw_i = float(sel_yaw[i])

            m = Marker()
            m.header.frame_id, m.header.stamp = self.frame_id, stamp
            m.ns, m.id, m.type, m.action = 'speed_footprints', m_id, Marker.CUBE, Marker.ADD
            m_id += 1
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(selected_xy[i, 0]), float(selected_xy[i, 1]), 0.15
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw_i / 2.0), math.cos(yaw_i / 2.0)
            m.scale.x, m.scale.y, m.scale.z = float(self.vehicle_length), float(self.vehicle_width), 0.05

            norm = float(np.clip(sp_kmh / max(1.0, self.max_speed_kmh), 0.0, 1.0))
            m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 0.1, 0.1, 0.6) if sp_kmh < 5.0 else (float(np.clip(2.0 * (1.0 - norm), 0.0, 1.0)), float(np.clip(2.0 * norm, 0.0, 1.0)), 0.15, 0.45)
            speed_array.markers.append(m)

            tm = Marker()
            tm.header.frame_id, tm.header.stamp = self.frame_id, stamp
            tm.ns, tm.id, tm.type, tm.action = 'speed_text', m_id, Marker.TEXT_VIEW_FACING, Marker.ADD
            m_id += 1
            tm.pose.position.x, tm.pose.position.y, tm.pose.position.z = float(selected_xy[i, 0]), float(selected_xy[i, 1]), 0.8
            tm.scale.z = 0.7
            tm.color.r, tm.color.g, tm.color.b, tm.color.a = 1.0, 1.0, 1.0, 1.0
            tm.text = f"{sp_kmh:.0f} km/h"
            speed_array.markers.append(tm)

        self.speed_pub.publish(speed_array)


if __name__ == '__main__':
    try:
        planner = LocalPathPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
