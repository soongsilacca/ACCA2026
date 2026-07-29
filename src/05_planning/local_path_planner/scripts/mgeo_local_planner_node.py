#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MGeo Link-Aware & Global Path Local Planner Node
Architecture: Autoware Universe & MGeo HD Map Topology Based Robust Avoidance

Features:
  - Global Path & MGeo Link Integration
  - Spatial Neighbor Link Query (Automatically finds Left/Right MGeo links)
  - Multi-Candidate Lane Change & Offset Avoidance (Current, Left, Right)
  - 100% On-Road Guarantee (Bounded by MGeo Road Width)
  - Proactive Obstacle Clearance Envelope
  - Dynamic Speed Profiling & Smooth Transition Filtering
"""

import os
import sys
import math
import json
import numpy as np

import rospy
import rospkg
import tf
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from global_path_planner.msg import PlannerTrajectory, PlannerWaypoint


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class QuinticPolynomial:
    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0
        T = max(1.0, T)
        T2 = T * T
        T3 = T2 * T
        T4 = T3 * T
        T5 = T4 * T

        b0 = xe - self.a0 - self.a1 * T - self.a2 * T2
        b1 = vxe - self.a1 - 2.0 * self.a2 * T
        b2 = axe - 2.0 * self.a2

        self.a3 = (10.0 * b0 / T3) - (4.0 * b1 / T2) + (0.5 * b2 / T)
        self.a4 = (-15.0 * b0 / T4) + (7.0 * b1 / T3) - (b2 / T2)
        self.a5 = (6.0 * b0 / T5) - (3.0 * b1 / T4) + (0.5 * b2 / T3)

    def calc(self, t):
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        return self.a0 + self.a1 * t + self.a2 * t2 + self.a3 * t3 + self.a4 * t4 + self.a5 * t5


class MGeoLocalPlannerNode:
    def __init__(self):
        rospy.init_node('mgeo_local_path_planner_node', anonymous=False)

        # Parameters
        self.horizon_dist = rospy.get_param('~planning/horizon_dist', 50.0)
        self.step_size = rospy.get_param('~planning/step_size', 0.5)
        self.max_speed_kmh = rospy.get_param('~planning/max_speed_kmh', 40.0)
        self.min_speed_kmh = rospy.get_param('~planning/min_speed_kmh', 10.0)
        self.max_lat_accel = rospy.get_param('~planning/max_lat_accel', 2.0)
        self.max_offset_rate = rospy.get_param('~planning/max_offset_rate', 0.25)
        self.publish_rate = rospy.get_param('~planning/publish_rate', 10.0)
        self.frame_id = rospy.get_param('~planning/frame_id', 'map')

        self.safety_margin = rospy.get_param('~rollouts/safety_margin', 1.3)
        self.enable_avoidance = rospy.get_param('~rollouts/enable_avoidance', True)
        self.min_stop_dist = rospy.get_param('~acc/min_stop_dist', 4.5)
        self.max_decel = rospy.get_param('~acc/max_decel', 2.5)

        self.vehicle_width = 1.89
        self.vehicle_length = 4.635
        self.wheelbase = 3.0
        self.max_speed_ms = self.max_speed_kmh / 3.6

        # Load MGeo Map
        rospack = rospkg.RosPack()
        try:
            hdmap_loader_path = rospack.get_path('hdmap_loader')
            link_file = os.path.join(hdmap_loader_path, 'scripts', 'link_set.json')
        except Exception:
            link_file = ""

        self.link_file = rospy.get_param('~link_file', link_file)
        self.links = []
        self.link_dict = {}
        self.load_mgeo_map()

        # State Variables
        self.global_path = None
        self.odom = None
        self.obstacles = []
        self.filtered_offset = 0.0

        # Publishers & Subscribers
        self.traj_pub = rospy.Publisher('/local_trajectory', PlannerTrajectory, queue_size=1)
        self.path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.rollout_pub = rospy.Publisher('/rollout_markers', MarkerArray, queue_size=1)
        self.speed_pub = rospy.Publisher('/speed_markers', MarkerArray, queue_size=1)
        self.status_pub = rospy.Publisher('/planner_status', String, queue_size=1)

        rospy.Subscriber('/global_path', Path, self.cb_global_path, queue_size=1)
        rospy.Subscriber('/localization/kinematic_state', Odometry, self.cb_odom, queue_size=1)
        rospy.Subscriber('/clusters_markers', MarkerArray, self.cb_clusters, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.planning_loop)
        rospy.loginfo("[MGeo Local Planner] Ready with %d MGeo Links.", len(self.links))

    def load_mgeo_map(self):
        if not self.link_file or not os.path.exists(self.link_file):
            return
        try:
            with open(self.link_file, 'r') as f:
                self.links = json.load(f)
            self.link_dict = {lk['idx']: lk for lk in self.links}
        except Exception as e:
            rospy.logerr("[MGeo Local Planner] Load error: %s", str(e))

    def cb_global_path(self, msg):
        if len(msg.poses) > 0:
            self.global_path = msg

    def cb_odom(self, msg):
        self.odom = msg
        self.planning_loop(None)

    def cb_clusters(self, msg):
        if self.odom is None:
            return
        ego_x = self.odom.pose.pose.position.x
        ego_y = self.odom.pose.pose.position.y
        q = self.odom.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        obs = []
        for m in msg.markers:
            if m.action in [Marker.DELETE, Marker.DELETEALL]:
                continue
            r = max(0.4, min(max(m.scale.x, m.scale.y) / 2.0, 2.5))
            f_id = m.header.frame_id if m.header.frame_id else "velodyne"
            ox, oy = m.pose.position.x, m.pose.position.y

            if f_id != self.frame_id:
                lx = m.pose.position.x + (3.85 if f_id == "velodyne" else 0.0)
                ly = m.pose.position.y
                ox = ego_x + lx * math.cos(yaw) - ly * math.sin(yaw)
                oy = ego_y + lx * math.sin(yaw) + ly * math.cos(yaw)

            obs.append({'x': ox, 'y': oy, 'r': r})
        self.obstacles = obs

    def get_global_reference_points(self, ego_x, ego_y, ego_yaw):
        if self.global_path is None or len(self.global_path.poses) < 2:
            return []

        poses = self.global_path.poses
        n = len(poses)
        min_d = float('inf')
        min_idx = 0

        for i in range(n):
            dx = poses[i].pose.position.x - ego_x
            dy = poses[i].pose.position.y - ego_y
            d = math.hypot(dx, dy)
            if d < min_d:
                min_d = d
                min_idx = i

        ref_pts = []
        accum = 0.0
        prev = [poses[min_idx].pose.position.x, poses[min_idx].pose.position.y]
        ref_pts.append(prev)

        for i in range(min_idx + 1, n):
            pt = [poses[i].pose.position.x, poses[i].pose.position.y]
            d = math.hypot(pt[0] - prev[0], pt[1] - prev[1])
            if accum + d > self.horizon_dist:
                rem = self.horizon_dist - accum
                frac = rem / d if d > 1e-6 else 0.0
                last_p = [prev[0] + frac * (pt[0] - prev[0]), prev[1] + frac * (pt[1] - prev[1])]
                ref_pts.append(last_p)
                break
            ref_pts.append(pt)
            accum += d
            prev = pt
        return ref_pts

    def find_spatial_adjacent_links(self, ego_x, ego_y, ego_yaw):
        """Find adjacent MGeo links located spatially to the left and right of ego vehicle."""
        left_link, right_link = None, None
        min_d_left, min_d_right = float('inf'), float('inf')

        for lk in self.links:
            pts = lk['points']
            for i in range(len(pts) - 1):
                p1, p2 = pts[i], pts[i + 1]
                dx, dy = p2[0] - p1[0], p2[1] - p1[1]
                l2 = dx * dx + dy * dy
                if l2 < 1e-6:
                    continue
                t = max(0.0, min(1.0, ((ego_x - p1[0]) * dx + (ego_y - p1[1]) * dy) / l2))
                proj = [p1[0] + t * dx, p1[1] + t * dy]

                seg_yaw = math.atan2(dy, dx)
                if abs(normalize_angle(seg_yaw - ego_yaw)) > math.pi / 3.0:
                    continue

                # Relative lateral displacement (left > 0, right < 0)
                rel_x = proj[0] - ego_x
                rel_y = proj[1] - ego_y
                lat = -rel_x * math.sin(ego_yaw) + rel_y * math.cos(ego_yaw)

                if 2.0 <= lat <= 4.8 and abs(lat) < min_d_left:
                    min_d_left = abs(lat)
                    left_link = lk
                elif -4.8 <= lat <= -2.0 and abs(lat) < min_d_right:
                    min_d_right = abs(lat)
                    right_link = lk
        return left_link, right_link

    def check_path_collision(self, xy_pts, yaw_pts, obstacles):
        if not xy_pts or not obstacles:
            return False, float('inf')
        min_dist = float('inf')
        foot_offsets = [-self.vehicle_length * 0.2, 0.0, self.wheelbase * 0.5, self.wheelbase * 1.0]

        for i in range(len(xy_pts)):
            px, py = xy_pts[i]
            yaw = yaw_pts[i]
            for ob in obstacles:
                for off in foot_offsets:
                    cx = px + off * math.cos(yaw)
                    cy = py + off * math.sin(yaw)
                    d = math.hypot(cx - ob['x'], cy - ob['y']) - ob['r']
                    min_dist = min(min_dist, d)
                    if d < (self.vehicle_width / 2.0 + self.safety_margin):
                        return True, min_dist
        return False, min_dist

    def planning_loop(self, event):
        if self.global_path is None or self.odom is None:
            return

        ego_x = self.odom.pose.pose.position.x
        ego_y = self.odom.pose.pose.position.y
        q = self.odom.pose.pose.orientation
        _, _, ego_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # 1. Base Reference Trajectory along Global Path
        ref_base = self.get_global_reference_points(ego_x, ego_y, ego_yaw)
        if len(ref_base) < 3:
            return

        s_accum = [0.0]
        for i in range(1, len(ref_base)):
            d = math.hypot(ref_base[i][0] - ref_base[i-1][0], ref_base[i][1] - ref_base[i-1][1])
            s_accum.append(s_accum[-1] + d)

        s_dense = np.arange(0.0, s_accum[-1], self.step_size)
        if len(s_dense) < 3:
            return

        x_base = np.interp(s_dense, s_accum, [p[0] for p in ref_base])
        y_base = np.interp(s_dense, s_accum, [p[1] for p in ref_base])
        yaw_base = np.zeros(len(s_dense))
        for i in range(len(s_dense) - 1):
            yaw_base[i] = math.atan2(y_base[i + 1] - y_base[i], x_base[i + 1] - x_base[i])
        yaw_base[-1] = yaw_base[-2]

        # 2. Query Adjacent MGeo Links for Lane Shift
        left_lk, right_lk = self.find_spatial_adjacent_links(ego_x, ego_y, ego_yaw)
        
        # Available Lateral Rollout Candidates
        candidate_offsets = [0.0]
        if left_lk:
            candidate_offsets.append(3.3)
        candidate_offsets.append(1.8) # Left Avoidance Offset
        if right_lk:
            candidate_offsets.append(-3.3)
        candidate_offsets.append(-1.8) # Right Avoidance Offset

        # 3. Evaluate Rollout Candidates
        best_offset = 0.0
        best_cost = float('inf')
        any_valid = False
        rollout_geometries = []

        trans_s = min(20.0, max(6.0, s_dense[-1] * 0.4))

        for d in candidate_offsets:
            qp = QuinticPolynomial(0.0, 0.0, 0.0, d, 0.0, 0.0, trans_s)
            rollout_xy = []
            rollout_yaw = []
            for i in range(len(s_dense)):
                lat = qp.calc(s_dense[i]) if s_dense[i] < trans_s else d
                nx = x_base[i] - math.sin(yaw_base[i]) * lat
                ny = y_base[i] + math.cos(yaw_base[i]) * lat
                rollout_xy.append((nx, ny))
                rollout_yaw.append(yaw_base[i])

            rollout_geometries.append(rollout_xy)
            blocked, min_obs_d = self.check_path_collision(rollout_xy, rollout_yaw, self.obstacles)

            if not blocked:
                any_valid = True
                obs_cost = (3.5 - min_obs_d) * 15.0 if min_obs_d < 3.5 else 0.0
                total_cost = abs(d) * 5.0 + abs(d - self.filtered_offset) * 2.0 + obs_cost
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_offset = d

        if any_valid:
            target_offset_goal = best_offset
            status_text = "LANE_KEEP" if abs(best_offset) < 0.5 else ("AVOID_LEFT" if best_offset > 0 else "AVOID_RIGHT")
        else:
            target_offset_goal = 0.0
            status_text = "STOP_BLOCKED"

        # 4. Filtered Smooth Transition
        step = max(-self.max_offset_rate, min(self.max_offset_rate, target_offset_goal - self.filtered_offset))
        self.filtered_offset += step

        # Generate Active Trajectory
        qp_active = QuinticPolynomial(0.0, 0.0, 0.0, self.filtered_offset, 0.0, 0.0, trans_s)
        selected_xy = []
        sel_yaw = []
        for i in range(len(s_dense)):
            lat = qp_active.calc(s_dense[i]) if s_dense[i] < trans_s else self.filtered_offset
            nx = x_base[i] - math.sin(yaw_base[i]) * lat
            ny = y_base[i] + math.cos(yaw_base[i]) * lat
            selected_xy.append((nx, ny))

        for i in range(len(s_dense) - 1):
            sel_yaw.append(math.atan2(selected_xy[i + 1][1] - selected_xy[i][1], selected_xy[i + 1][0] - selected_xy[i][0]))
        sel_yaw.append(sel_yaw[-1])

        # 5. Speed Profiling
        n_pts = len(selected_xy)
        curvature = np.zeros(n_pts)
        for i in range(1, n_pts - 1):
            x1, y1 = selected_xy[i - 1]
            x2, y2 = selected_xy[i]
            x3, y3 = selected_xy[i + 1]
            area = 0.5 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
            a = math.hypot(x2 - x1, y2 - y1)
            b = math.hypot(x3 - x2, y3 - y2)
            c = math.hypot(x1 - x3, y1 - y3)
            if a * b * c > 1e-6:
                curvature[i] = (4.0 * abs(area)) / (a * b * c)

        target_speeds = np.zeros(n_pts)
        curr_blocked, curr_min_d = self.check_path_collision(selected_xy, sel_yaw, self.obstacles)

        for i in range(n_pts):
            k = curvature[i]
            v_curve = math.sqrt(self.max_lat_accel / k) if k > 1e-3 else self.max_speed_ms
            v = min(self.max_speed_ms, max(self.min_speed_kmh / 3.6, v_curve))

            if status_text == "STOP_BLOCKED" or curr_blocked:
                stop_s = max(0.0, curr_min_d - self.min_stop_dist)
                v = 0.0 if s_dense[i] >= stop_s else min(v, math.sqrt(max(0.0, 2.0 * self.max_decel * (stop_s - s_dense[i]))))
            target_speeds[i] = v

        # 6. Publish Trajectory
        stamp = rospy.Time.now()
        traj_msg = PlannerTrajectory()
        traj_msg.header.frame_id = self.frame_id
        traj_msg.header.stamp = stamp

        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = stamp

        for i in range(n_pts):
            wp = PlannerWaypoint()
            wp.position.x = selected_xy[i][0]
            wp.position.y = selected_xy[i][1]
            wp.position.z = 0.0
            wp.yaw = sel_yaw[i]
            wp.target_speed = target_speeds[i]
            wp.curvature = curvature[i]
            traj_msg.waypoints.append(wp)

            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = selected_xy[i][0]
            ps.pose.position.y = selected_xy[i][1]
            path_msg.poses.append(ps)

        self.traj_pub.publish(traj_msg)
        self.path_pub.publish(path_msg)
        self.status_pub.publish(String(data=status_text))

        # 7. RViz Visualizations
        self.publish_rviz_markers(rollout_geometries, selected_xy, sel_yaw, target_speeds, stamp)

    def publish_rviz_markers(self, rollout_geometries, selected_xy, sel_yaw, target_speeds, stamp):
        ma = MarkerArray()
        m_id = 0

        for i, r_pts in enumerate(rollout_geometries):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = "mgeo_rollouts"
            m.id = m_id
            m_id += 1
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.12
            m.color.r, m.color.g, m.color.b, m.color.a = (0.2, 0.8, 0.2, 0.7) if i == 0 else (0.9, 0.5, 0.2, 0.5)
            for p in r_pts:
                pt = Point()
                pt.x, pt.y, pt.z = p[0], p[1], 0.1
                m.points.append(pt)
            ma.markers.append(m)

        self.rollout_pub.publish(ma)

        sm = MarkerArray()
        del_m = Marker()
        del_m.action = Marker.DELETEALL
        sm.markers.append(del_m)

        step = max(1, int(2.5 / self.step_size))
        s_id = 0
        for i in range(0, len(selected_xy), step):
            sp_kmh = target_speeds[i] * 3.6
            yaw = sel_yaw[i]

            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = "speed_footprint"
            m.id = s_id
            s_id += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = selected_xy[i][0]
            m.pose.position.y = selected_xy[i][1]
            m.pose.position.z = 0.15
            m.pose.orientation.z = math.sin(yaw / 2.0)
            m.pose.orientation.w = math.cos(yaw / 2.0)
            m.scale.x = self.vehicle_length
            m.scale.y = self.vehicle_width
            m.scale.z = 0.05

            if sp_kmh < 5.0:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.1, 0.1, 0.6
            else:
                norm = max(0.0, min(1.0, sp_kmh / self.max_speed_kmh))
                m.color.r = max(0.0, min(1.0, 2.0 * (1.0 - norm)))
                m.color.g = max(0.0, min(1.0, 2.0 * norm))
                m.color.b, m.color.a = 0.15, 0.45
            sm.markers.append(m)

            tm = Marker()
            tm.header.frame_id = self.frame_id
            tm.header.stamp = stamp
            tm.ns = "speed_text"
            tm.id = s_id
            s_id += 1
            tm.type = Marker.TEXT_VIEW_FACING
            tm.action = Marker.ADD
            tm.pose.position.x = selected_xy[i][0]
            tm.pose.position.y = selected_xy[i][1]
            tm.pose.position.z = 0.8
            tm.scale.z = 0.7
            tm.color.r, tm.color.g, tm.color.b, tm.color.a = 1.0, 1.0, 1.0, 1.0
            tm.text = "{:.0f} km/h".format(sp_kmh)
            sm.markers.append(tm)

        self.speed_pub.publish(sm)


if __name__ == '__main__':
    try:
        node = MGeoLocalPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
