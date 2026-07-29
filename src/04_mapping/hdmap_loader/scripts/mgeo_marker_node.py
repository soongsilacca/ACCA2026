#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mgeo_marker_node.py

MGeo Map files (node_set.json, link_set.json, global_info.json)를 읽어
RViz MarkerArray로 publish하는 노드.

주요 기능:
  - node_set.json: SPHERE_LIST를 이용하여 노드 시각화 (일반 노드, 정지선 노드, 신호등 노드 색상 구분)
  - link_set.json: LINE_STRIP을 이용하여 링크 시각화 (우회전, 좌회전, 직진, 일반 차선 색상 구분)
  - Lane Boundary: 각 링크의 도로폭(width_start, width_end)을 이용해 좌/우 차선 경계선(Lane Boundary) 시각화
  - 방향성 표시: 링크 중간 지점에 방향 화살표(ARROW) 추가 (선택 가능)
  - IMU 등 타 센서 정렬 없이 JSON 원본 좌표계를 그대로 사용
"""

import os
import sys
import json
import math
import rospy
import rospkg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


def make_color(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = r, g, b, a
    return c


def compute_lane_boundaries(pts, width_start, width_end, z_offset=0.0):
    """
    Calculates 3D left and right lane boundary points given link centerline points and width.
    """
    n = len(pts)
    if n < 2:
        return [], []

    ws = float(width_start) if (width_start is not None and float(width_start) > 0.5) else 3.5
    we = float(width_end) if (width_end is not None and float(width_end) > 0.5) else 3.5

    left_pts = []
    right_pts = []

    for i in range(n):
        if i == 0:
            dx = pts[1][0] - pts[0][0]
            dy = pts[1][1] - pts[0][1]
        elif i == n - 1:
            dx = pts[-1][0] - pts[-2][0]
            dy = pts[-1][1] - pts[-2][1]
        else:
            dx = pts[i + 1][0] - pts[i - 1][0]
            dy = pts[i + 1][1] - pts[i - 1][1]

        yaw = math.atan2(dy, dx)
        nx  = -math.sin(yaw)
        ny  =  math.cos(yaw)

        w = ws + (we - ws) * (i / float(n - 1)) if n > 1 else ws
        half_w = w / 2.0

        px, py, pz = pts[i][0], pts[i][1], pts[i][2] + z_offset

        left_pts.append(Point(x=px + nx * half_w, y=py + ny * half_w, z=pz))
        right_pts.append(Point(x=px - nx * half_w, y=py - ny * half_w, z=pz))

    return left_pts, right_pts


class MGeoMarkerNode:
    def __init__(self):
        rospy.init_node('mgeo_marker_node', anonymous=False)

        # 1. ROS Parameters
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('hdmap_loader')

        self.node_file = rospy.get_param(
            '~node_file', os.path.join(package_path, 'scripts', 'node_set.json')
        )
        self.link_file = rospy.get_param(
            '~link_file', os.path.join(package_path, 'scripts', 'link_set.json')
        )
        self.global_info_file = rospy.get_param(
            '~global_info_file', os.path.join(package_path, 'scripts', 'global_info.json')
        )

        self.frame_id        = rospy.get_param('~frame_id', 'map')
        self.publish_hz      = rospy.get_param('~publish_hz', 1.0)
        self.show_arrows     = rospy.get_param('~show_arrows', True)
        self.show_boundaries = rospy.get_param('~show_boundaries', True)
        self.z_offset        = rospy.get_param('~z_offset', 0.0)

        # State Variables
        self.raw_nodes = []
        self.raw_links = []
        self.local_origin = [0.0, 0.0, 0.0]

        # Load MGeo Data
        self.load_mgeo_data()

        # Publisher (latch=True)
        self.marker_pub = rospy.Publisher('/mgeo_markers', MarkerArray, queue_size=1, latch=True)

        rospy.loginfo("[MGeo Viz] Publishing map with raw JSON coordinates and Lane Boundaries.")
        self.publish_markers()

    def load_mgeo_data(self):
        # Read Global Info
        if os.path.exists(self.global_info_file):
            try:
                with open(self.global_info_file, 'r') as f:
                    global_info = json.load(f)
                self.local_origin = global_info.get('local_origin_in_global', [0.0, 0.0, 0.0])
                rospy.loginfo(f"[MGeo Viz] Loaded global origin: {self.local_origin}")
            except Exception as e:
                rospy.logwarn(f"[MGeo Viz] Failed to parse global_info.json: {e}")
        else:
            rospy.logwarn(f"[MGeo Viz] global_info.json not found at {self.global_info_file}")

        # Read Nodes
        if os.path.exists(self.node_file):
            try:
                with open(self.node_file, 'r') as f:
                    self.raw_nodes = json.load(f)
                rospy.loginfo(f"[MGeo Viz] Loaded {len(self.raw_nodes)} nodes.")
            except Exception as e:
                rospy.logerr(f"[MGeo Viz] Failed to parse node_set.json: {e}")
                sys.exit(1)
        else:
            rospy.logerr(f"[MGeo Viz] node_set.json not found at {self.node_file}")
            sys.exit(1)

        # Read Links
        if os.path.exists(self.link_file):
            try:
                with open(self.link_file, 'r') as f:
                    self.raw_links = json.load(f)
                rospy.loginfo(f"[MGeo Viz] Loaded {len(self.raw_links)} links.")
            except Exception as e:
                rospy.logerr(f"[MGeo Viz] Failed to parse link_set.json: {e}")
                sys.exit(1)
        else:
            rospy.logerr(f"[MGeo Viz] link_set.json not found at {self.link_file}")
            sys.exit(1)

    def publish_markers(self):
        ma = MarkerArray()
        stamp_time = rospy.Time(0)

        # ──────────────────────────────────────────────
        # 1. Visualize Nodes (using SPHERE_LIST for efficiency)
        # ──────────────────────────────────────────────
        node_marker = Marker()
        node_marker.header.frame_id = self.frame_id
        node_marker.header.stamp = stamp_time
        node_marker.ns = "nodes"
        node_marker.id = 0
        node_marker.type = Marker.SPHERE_LIST
        node_marker.action = Marker.ADD
        node_marker.scale.x = 1.0
        node_marker.scale.y = 1.0
        node_marker.scale.z = 1.0
        node_marker.pose.orientation.w = 1.0
        node_marker.lifetime = rospy.Duration(0)

        color_default = make_color(0.2, 0.7, 0.9, 0.5)      # Cyan
        color_stop    = make_color(0.95, 0.1, 0.1, 0.9)     # Red
        color_traffic = make_color(0.98, 0.5, 0.0, 0.9)     # Orange

        for n in self.raw_nodes:
            pt = n.get('point')
            if not pt or len(pt) < 3:
                continue
            
            p = Point(x=pt[0], y=pt[1], z=pt[2] + self.z_offset)
            node_marker.points.append(p)

            if n.get('on_stop_line', False):
                node_marker.colors.append(color_stop)
            elif n.get('traffic_light_id') is not None:
                node_marker.colors.append(color_traffic)
            else:
                node_marker.colors.append(color_default)

        ma.markers.append(node_marker)

        # ──────────────────────────────────────────────
        # 2. Visualize Links & Lane Boundaries
        # ──────────────────────────────────────────────
        color_straight    = make_color(0.15, 0.55, 0.95, 0.8)   # Electric Blue
        color_left        = make_color(0.85, 0.25, 0.9, 0.8)   # Neon Purple
        color_unprotected = make_color(0.95, 0.6, 0.05, 0.8)   # Amber
        color_normal      = make_color(0.5, 0.6, 0.7, 0.6)    # Slate Blue/Grey

        # Boundary Colors
        color_bound_left  = make_color(0.95, 0.85, 0.2, 0.65)  # Translucent Yellow
        color_bound_right = make_color(0.95, 0.95, 0.95, 0.65) # Translucent White

        link_id_counter = 1

        for link in self.raw_links:
            pts = link.get('points')
            if not pts or len(pts) < 2:
                continue

            # Centerline Marker
            link_marker = Marker()
            link_marker.header.frame_id = self.frame_id
            link_marker.header.stamp = stamp_time
            link_marker.ns = "links"
            link_marker.id = link_id_counter
            link_marker.type = Marker.LINE_STRIP
            link_marker.action = Marker.ADD
            link_marker.scale.x = 0.35
            link_marker.scale.y = 0.35
            link_marker.scale.z = 0.35
            link_marker.pose.orientation.w = 1.0
            link_marker.lifetime = rospy.Duration(0)

            signal = link.get('related_signal')
            if signal == 'straight':
                c = color_straight
            elif signal == 'left' or signal == 'uturn_normal':
                c = color_left
            elif signal in ['right_unprotected', 'left_unprotected']:
                c = color_unprotected
            else:
                c = color_normal

            link_marker.color = c

            for pt in pts:
                p = Point(x=pt[0], y=pt[1], z=pt[2] + self.z_offset)
                link_marker.points.append(p)

            ma.markers.append(link_marker)

            # ──────────────────────────────────────────────
            # 3. Calculate & Visualize Lane Boundaries
            # ──────────────────────────────────────────────
            if self.show_boundaries:
                w_start = link.get('width_start')
                w_end   = link.get('width_end')
                left_pts, right_pts = compute_lane_boundaries(pts, w_start, w_end, self.z_offset)

                if left_pts and right_pts:
                    # Left Lane Boundary
                    lb_left = Marker()
                    lb_left.header.frame_id = self.frame_id
                    lb_left.header.stamp = stamp_time
                    lb_left.ns = "lane_boundaries_left"
                    lb_left.id = link_id_counter
                    lb_left.type = Marker.LINE_STRIP
                    lb_left.action = Marker.ADD
                    lb_left.scale.x = 0.15
                    lb_left.pose.orientation.w = 1.0
                    lb_left.color = color_bound_left
                    lb_left.points = left_pts
                    ma.markers.append(lb_left)

                    # Right Lane Boundary
                    lb_right = Marker()
                    lb_right.header.frame_id = self.frame_id
                    lb_right.header.stamp = stamp_time
                    lb_right.ns = "lane_boundaries_right"
                    lb_right.id = link_id_counter
                    lb_right.type = Marker.LINE_STRIP
                    lb_right.action = Marker.ADD
                    lb_right.scale.x = 0.15
                    lb_right.pose.orientation.w = 1.0
                    lb_right.color = color_bound_right
                    lb_right.points = right_pts
                    ma.markers.append(lb_right)

            # ──────────────────────────────────────────────
            # 4. Add Direction Arrow for each link
            # ──────────────────────────────────────────────
            if self.show_arrows and len(pts) >= 2:
                mid_idx = len(pts) // 2
                p1 = pts[mid_idx - 1]
                p2 = pts[mid_idx]

                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                dz = p2[2] - p1[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                if dist > 0.1:
                    arrow_marker = Marker()
                    arrow_marker.header.frame_id = self.frame_id
                    arrow_marker.header.stamp = stamp_time
                    arrow_marker.ns = "arrows"
                    arrow_marker.id = link_id_counter
                    arrow_marker.type = Marker.ARROW
                    arrow_marker.action = Marker.ADD
                    arrow_marker.lifetime = rospy.Duration(0)

                    arrow_marker.scale.x = 0.25
                    arrow_marker.scale.y = 0.6
                    arrow_marker.scale.z = 0.7
                    arrow_marker.color = make_color(0.9, 0.9, 0.9, 0.6)

                    arrow_len = min(1.5, dist)
                    nx, ny, nz = dx / dist, dy / dist, dz / dist

                    ap1 = Point(x=p1[0], y=p1[1], z=p1[2] + self.z_offset)
                    ap2 = Point(x=p1[0] + nx * arrow_len, y=p1[1] + ny * arrow_len, z=p1[2] + nz * arrow_len + self.z_offset)

                    arrow_marker.points.append(ap1)
                    arrow_marker.points.append(ap2)

                    ma.markers.append(arrow_marker)

            link_id_counter += 1

        self.marker_pub.publish(ma)

    def run(self):
        rate = rospy.Rate(self.publish_hz)
        while not rospy.is_shutdown():
            self.publish_markers()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = MGeoMarkerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
