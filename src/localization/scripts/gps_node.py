#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import utm
import json
import os
import re
import rospkg
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class GPSNode:
    def __init__(self):
        rospy.init_node('gps_node', anonymous=True)

        # 1. 초기값 설정
        self.use_map_anchor = False
        self.anchor_east = 0.0
        self.anchor_north = 0.0
        self.anchor_zone = None
        self.first_gps = True
        self._origin_east = 0.0
        self._origin_north = 0.0
        self.initial_yaw = None
        self.rotate_by_initial_yaw = rospy.get_param('~rotate_by_initial_yaw', False)

        # 2. MGeo와 동일한 global_info.json에서 map 원점을 로드한다.
        #    MGeo JSON의 point는 local_origin_in_global을 뺀 UTM 좌표이므로
        #    GPS도 반드시 같은 값을 빼야 지도와 일치한다.
        try:
            rospack = rospkg.RosPack()
            default_global_info = os.path.join(
                rospack.get_path('map_viz'), 'scripts', 'global_info.json'
            )
            global_info_path = rospy.get_param('~global_info_file', default_global_info)

            if os.path.exists(global_info_path):
                with open(global_info_path, 'r') as f:
                    global_info = json.load(f)

                origin = global_info.get('local_origin_in_global')
                if not isinstance(origin, list) or len(origin) < 2:
                    raise ValueError("local_origin_in_global must contain easting and northing")

                coordinate_system = global_info.get('global_coordinate_system', '')
                zone_match = re.search(r'(?:^|\s)\+zone=(\d+)(?:\s|$)', coordinate_system)
                if zone_match:
                    self.anchor_zone = int(zone_match.group(1))

                self.anchor_east = float(origin[0])
                self.anchor_north = float(origin[1])
                self.use_map_anchor = True
                rospy.loginfo(
                    "GPS Node: MGeo map origin ENABLED (E:%.3f, N:%.3f, file:%s)",
                    self.anchor_east, self.anchor_north, global_info_path
                )
            else:
                rospy.logwarn(f"GPS Node: global_info.json not found at {global_info_path}")

        except Exception as e:
            rospy.logerr(f"GPS Node: Error loading MGeo map origin: {e}")

        # 3. Subscriber & Publisher
        # /gps 는 morai_udp_bridge/gps_publisher_node 가 sensor_msgs/NavSatFix 로 발행
        self.sub = rospy.Subscriber('/gps', NavSatFix, self.gps_callback)
        self.sub_imu = rospy.Subscriber('/imu', Imu, self.imu_callback)

        # PoseWithCovarianceStamped: 단독 사용 시 (Local mode)
        self.pose_pub = rospy.Publisher('/gps_pose', PoseWithCovarianceStamped, queue_size=10)

        # Odometry: Global EKF의 odom0 입력용 (회전 무관, 위치만 보정)
        self.odom_pub = rospy.Publisher('/gps_odom', Odometry, queue_size=10)

    def imu_callback(self, msg):
        if self.initial_yaw is None:
            import math
            q = msg.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.initial_yaw = math.atan2(siny_cosp, cosy_cosp)
            rospy.loginfo(f"GPS Node: Initial IMU Yaw received: {self.initial_yaw * 180.0 / math.pi:.2f} degrees")

    def gps_callback(self, msg: NavSatFix):
        # NavSatFix: latitude, longitude, altitude, status, position_covariance
        stamp = msg.header.stamp

        # NO FIX → 무시
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            rospy.logwarn_throttle(5.0, "GPS Node: No fix. Skipping.")
            return

        # Reset if time goes backwards (e.g. bag loop/rewind)
        if hasattr(self, 'last_gps_time') and stamp.to_sec() < self.last_gps_time:
            self.first_gps = True
            self.initial_yaw = None
        self.last_gps_time = stamp.to_sec()

        # 1. 현재 위경도를 UTM 좌표로 변환
        easting, northing, gps_zone, _ = utm.from_latlon(msg.latitude, msg.longitude)
        if self.anchor_zone is not None and gps_zone != self.anchor_zone:
            rospy.logerr_throttle(
                5.0,
                "GPS Node: GPS UTM zone (%d) does not match MGeo zone (%d).",
                gps_zone, self.anchor_zone
            )
            return

        # 2. Origin 설정
        if self.use_map_anchor:
            # global_info.json에서 지정한 MGeo 고정 좌표를 origin으로 사용
            dx = easting - self.anchor_east
            dy = northing - self.anchor_north
            if self.first_gps:
                self.first_gps = False
                rospy.loginfo(f"GPS Node: Using map anchor as origin. First offset: dx={dx:.2f}, dy={dy:.2f}")
        else:
            # map anchor를 읽지 못한 경우 → 첫 GPS 수신 위치를 origin으로
            if self.first_gps:
                self._origin_east = easting
                self._origin_north = northing
                self.first_gps = False
                rospy.logwarn(
                    "GPS Node: No map anchor. Using first GPS position as origin (E:%.3f, N:%.3f)",
                    easting, northing
                )
            dx = easting - self._origin_east
            dy = northing - self._origin_north

        # Keep map coordinates fixed by default. Rotating GPS by initial yaw makes
        # the map frame vehicle-relative and can double-apply yaw in the global EKF.
        if self.rotate_by_initial_yaw and self.initial_yaw is not None:
            import math
            cos_yaw = math.cos(self.initial_yaw)
            sin_yaw = math.sin(self.initial_yaw)
            local_x = dx * cos_yaw + dy * sin_yaw
            local_y = -dx * sin_yaw + dy * cos_yaw
        else:
            local_x = dx
            local_y = dy

        altitude = msg.altitude

        # ── PoseWithCovarianceStamped (기존 호환용) ─────────────────────
        pose_stamped = PoseWithCovarianceStamped()
        pose_stamped.header.stamp = stamp
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.pose.position.x = local_x
        pose_stamped.pose.pose.position.y = local_y
        pose_stamped.pose.pose.position.z = altitude
        pose_stamped.pose.pose.orientation.w = 1.0

        cov = [0.0] * 36
        cov[0]  = msg.position_covariance[0]    # X
        cov[7]  = msg.position_covariance[4]    # Y
        cov[14] = 999.0  # Z (미사용)
        cov[21] = 999.0  # Roll (미사용)
        cov[28] = 999.0  # Pitch (미사용)
        cov[35] = 999.0  # Yaw (미사용)
        pose_stamped.pose.covariance = cov
        self.pose_pub.publish(pose_stamped)

        # ── Odometry (Global EKF odom0 입력용) ──────────────────────────
        gps_odom = Odometry()
        gps_odom.header.stamp = stamp
        gps_odom.header.frame_id = "map"
        gps_odom.child_frame_id = "base_link"
        gps_odom.pose.pose.position.x = local_x
        gps_odom.pose.pose.position.y = local_y
        gps_odom.pose.pose.position.z = 0.0
        gps_odom.pose.pose.orientation.w = 1.0

        cov_odom = [0.0] * 36
        cov_odom[0]  = msg.position_covariance[0]    # X
        cov_odom[7]  = msg.position_covariance[4]    # Y
        cov_odom[14] = 999.0  # Z (무시)
        cov_odom[21] = 999.0  # Roll (무시)
        cov_odom[28] = 999.0  # Pitch (무시)
        cov_odom[35] = 999.0  # Yaw
        gps_odom.pose.covariance = cov_odom
        self.odom_pub.publish(gps_odom)

if __name__ == '__main__':
    try:
        gps_node = GPSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
