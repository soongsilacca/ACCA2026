#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
static_path_publisher_node.py

Reads a static trajectory file (X Y Z coordinates) and publishes it as:
1. nav_msgs/Path to /global_path (for standard controllers)
2. global_path_planner/PlannerTrajectory to /global_trajectory (with curvature & MGeo speed profile)
"""

import os
import math
import json
import numpy as np
from scipy.spatial import cKDTree

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from global_path_planner.msg import PlannerTrajectory, PlannerWaypoint
from visualization_msgs.msg import Marker, MarkerArray


class StaticPathPublisher:
    def __init__(self):
        rospy.init_node('static_path_publisher_node', anonymous=False)

        # Configurable Parameters
        try:
            default_path = os.path.join(rospkg.RosPack().get_path('global_path_planner'), '2026_molit_comp_global_path.txt')
        except Exception:
            from pathlib import Path
            default_path = str(Path(__file__).resolve().parents[1] / '2026_molit_comp_global_path.txt')
        self.file_path = rospy.get_param('~file_path', default_path)
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.path_topic = rospy.get_param('~path_topic', '/global_path')
        self.traj_topic = rospy.get_param('~traj_topic', '/global_trajectory')
        self.publish_rate = rospy.get_param('~publish_rate', 0.5)  # Hz
        
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 4.5)  # Increased from 2.0 to reduce braking on curves
        self.default_speed_kmh = rospy.get_param('~default_speed_kmh', 60.0)
        self.min_speed_kmh = rospy.get_param('~min_speed_kmh', 15.0)

        # 1. Load MGeo Map
        self.links = []
        self.link_tree = None
        self.link_speeds = []
        self.load_mgeo_map()

        rospy.loginfo(f"[Path Publisher] Loading path from: {self.file_path}")
        if not os.path.exists(self.file_path):
            rospy.logerr(f"[Path Publisher] File does not exist: {self.file_path}")
            return

        # 2. Process Path & Generate Speeds
        self.path_msg, self.traj_msg, self.speed_markers = self.load_and_process_path()
        if self.path_msg is None:
            rospy.logerr("[Path Publisher] Failed to load path data.")
            return

        # Publishers
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.traj_pub = rospy.Publisher(self.traj_topic, PlannerTrajectory, queue_size=1, latch=True)
        self.speed_pub = rospy.Publisher('/global_speed_markers', MarkerArray, queue_size=1, latch=True)

        rospy.loginfo(f"[Path Publisher] Publishing to '{self.path_topic}' and '{self.traj_topic}'")
        
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
        self.timer_callback(None)

    def load_mgeo_map(self):
        rospack = rospkg.RosPack()
        try:
            hdmap_loader_path = rospack.get_path('hdmap_loader')
            link_file = os.path.join(hdmap_loader_path, 'scripts', 'link_set.json')
            if os.path.exists(link_file):
                with open(link_file, 'r') as f:
                    self.links = json.load(f)
                
                # Build KDTree of link center points for fast nearest neighbor lookup
                pts = []
                for lk in self.links:
                    p = lk['points'][len(lk['points'])//2] # approximate center
                    pts.append([p[0], p[1]])
                    
                    # Read MGeo properties
                    raw_speed = float(lk.get('max_speed', self.default_speed_kmh))
                    
                    road_type = str(lk.get('road_type', '0'))
                    link_type = str(lk.get('link_type', '0'))
                    
                    # 카테고리별 최대 허용 속도 (Max Limit)
                    if road_type == '2': # Tunnel
                        limit = 40.0
                    else:
                        # MORAI MGeo 데이터에서는 'road_type' 1이 일반도로와 고속도로에 모두 쓰임.
                        # 따라서 맵에 명시된 속도가 60을 초과하면 고속도로로 간주하고, 아니면 일반 도로로 간주함.
                        if raw_speed > 60.0:
                            limit = 120.0  # Highway
                        else:
                            limit = 60.0   # Normal
                            
                    # 특정 링크 타입(교차로 내부, 회전교차로, 요금소)인 경우 속도 제한 강화
                    if link_type in ['3', '4', '5']:
                        limit = 30.0

                    # 1. 기본적으로 MGeo에 적힌 속도를 따름 (limit을 넘지 않는 선에서)
                    # 2. 만약 MGeo 속도가 0이거나 너무 낮으면(오류), 해당 구간의 max limit을 기본값으로 부여
                    if raw_speed < 30.0:
                        speed = limit
                    else:
                        speed = min(raw_speed, limit)
                        
                    self.link_speeds.append(speed)
                
                if pts:
                    self.link_tree = cKDTree(pts)
                    rospy.loginfo(f"[Path Publisher] Loaded {len(pts)} MGeo links for speed limits.")
        except Exception as e:
            rospy.logwarn(f"[Path Publisher] Failed to load MGeo links, falling back to default speeds. ({e})")

    def get_mgeo_speed_limit(self, x, y):
        if self.link_tree is None:
            return self.default_speed_kmh
        _, idx = self.link_tree.query([x, y])
        return self.link_speeds[idx]

    def load_and_process_path(self):
        try:
            data = np.loadtxt(self.file_path)
            if data.ndim != 2 or data.shape[1] < 2:
                rospy.logerr("[Path Publisher] Invalid data format.")
                return None, None

            n_points = len(data)
            path_msg = Path()
            path_msg.header.frame_id = self.frame_id
            
            traj_msg = PlannerTrajectory()
            traj_msg.header.frame_id = self.frame_id
            
            speed_markers = MarkerArray()
            
            # 1. Compute curvature (use larger stride to reduce noise/jaggedness)
            curvature = np.zeros(n_points)
            step = 5
            for i in range(step, n_points - step):
                x1, y1 = data[i-step, 0], data[i-step, 1]
                x2, y2 = data[i, 0], data[i, 1]
                x3, y3 = data[i+step, 0], data[i+step, 1]
                area = 0.5 * (x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2))
                a = math.hypot(x2 - x1, y2 - y1)
                b = math.hypot(x3 - x2, y3 - y2)
                c = math.hypot(x1 - x3, y1 - y3)
                if a * b * c > 1e-6:
                    curvature[i] = (4.0 * abs(area)) / (a * b * c)
            
            # fill boundary curvatures
            if n_points > step:
                curvature[:step] = curvature[step]
                curvature[-step:] = curvature[-step-1]

            # moving average to smooth curvature spikes
            window_size = 15
            kernel = np.ones(window_size) / window_size
            curvature = np.convolve(curvature, kernel, mode='same')

            # 2. Pre-calculate speeds and backward smoothing
            raw_speeds = np.zeros(n_points)
            yaws = np.zeros(n_points)
            
            prev_yaw = 0.0
            for i in range(n_points):
                x = float(data[i, 0])
                y = float(data[i, 1])
                if i < n_points - 1:
                    dx = data[i+1, 0] - x
                    dy = data[i+1, 1] - y
                    yaw = math.atan2(dy, dx)
                else:
                    yaw = prev_yaw
                yaws[i] = yaw
                prev_yaw = yaw
                
                mgeo_kmh = self.get_mgeo_speed_limit(x, y)
                mgeo_ms = mgeo_kmh / 3.6
                k = curvature[i]
                
                # 곡률(k)에 비례하여 해당 구간의 maxspeed(mgeo_ms)에서 퍼센티지로 감속
                # k가 클수록(곡률이 심할수록) 속도 비율이 선형적으로 낮아짐 (최소 30% 보장)
                curvature_factor = 5.0
                speed_ratio = max(0.3, 1.0 - (k * curvature_factor))
                v_curve = mgeo_ms * speed_ratio
                
                raw_speeds[i] = min(mgeo_ms, max(self.min_speed_kmh / 3.6, v_curve))

            # Backward Smoothing (Decelerate earlier)
            smoothed_speeds = np.copy(raw_speeds)
            decel_rate = 1.5  # m/s^2 (comfort deceleration)
            for i in range(n_points - 2, -1, -1):
                dist = math.hypot(data[i+1, 0] - data[i, 0], data[i+1, 1] - data[i, 1])
                max_v = math.sqrt(smoothed_speeds[i+1]**2 + 2.0 * decel_rate * dist)
                smoothed_speeds[i] = min(smoothed_speeds[i], max_v)

            for i in range(n_points):
                x = float(data[i, 0])
                y = float(data[i, 1])
                yaw = yaws[i]
                target_speed = smoothed_speeds[i]

                # Build Path msg
                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                path_msg.poses.append(pose)

                # Build Traj msg
                wp = PlannerWaypoint()
                wp.position.x = x
                wp.position.y = y
                wp.yaw = yaw
                wp.target_speed = target_speed
                wp.curvature = curvature[i]
                traj_msg.waypoints.append(wp)

                # Build Marker (Sampled to avoid lagging RViz)
                if i % 5 == 0:
                    kmh = target_speed * 3.6
                    tm = Marker()
                    tm.header.frame_id = self.frame_id
                    tm.ns = "global_speed_text"
                    tm.id = i
                    tm.type = Marker.TEXT_VIEW_FACING
                    tm.action = Marker.ADD
                    tm.pose.position.x = x
                    tm.pose.position.y = y
                    tm.pose.position.z = 1.0
                    tm.scale.z = 0.8
                    tm.color.r, tm.color.g, tm.color.b, tm.color.a = 0.0, 1.0, 0.5, 1.0
                    tm.text = "{:.0f} km/h".format(kmh)
                    speed_markers.markers.append(tm)

            return path_msg, traj_msg, speed_markers

        except Exception as e:
            rospy.logerr(f"[Path Publisher] Error processing path: {e}")
            return None, None, None

    def timer_callback(self, event):
        if self.path_msg is not None and self.traj_msg is not None:
            now = rospy.Time.now()
            self.path_msg.header.stamp = now
            self.traj_msg.header.stamp = now
            for pose in self.path_msg.poses:
                pose.header.stamp = now
            for m in self.speed_markers.markers:
                m.header.stamp = now

            self.path_pub.publish(self.path_msg)
            self.traj_pub.publish(self.traj_msg)
            self.speed_pub.publish(self.speed_markers)


if __name__ == '__main__':
    try:
        StaticPathPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
