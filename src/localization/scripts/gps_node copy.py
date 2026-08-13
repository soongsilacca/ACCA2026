#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import utm
import yaml
import os
import rospkg
from morai_msgs.msg import GPSMessage
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

class GPSNode:
    def __init__(self):
        rospy.init_node('gps_node', anonymous=True)

        # 1. 초기값 설정
        self.use_map_anchor = False
        self.anchor_east = 0.0
        self.anchor_north = 0.0

        # 2. YAML 파일 직접 로드
        try:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('localization')
            yaml_path = os.path.join(package_path, 'config', 'map_anchor.yaml')

            if os.path.exists(yaml_path):
                with open(yaml_path, 'r') as f:
                    config = yaml.safe_load(f)

                if config.get('enabled', False):
                    self.use_map_anchor = True
                    anchor_lat = config['latitude']
                    anchor_lon = config['longitude']
                    self.anchor_east, self.anchor_north, _, _ = utm.from_latlon(anchor_lat, anchor_lon)
                    rospy.loginfo(f"GPS Node: Map Anchor ENABLED (Lat:{anchor_lat}, Lon:{anchor_lon})")
                else:
                    rospy.loginfo("GPS Node: Map Anchor is DISABLED in YAML.")
            else:
                rospy.logwarn(f"GPS Node: map_anchor.yaml not found at {yaml_path}")

        except Exception as e:
            rospy.logerr(f"GPS Node: Error loading map_anchor.yaml: {e}")

        # 3. Subscriber & Publisher
        self.sub = rospy.Subscriber('/gps', GPSMessage, self.gps_callback)

        # PoseWithCovarianceStamped: 단독 사용 시 (Local mode)
        self.pose_pub = rospy.Publisher('/gps_pose', PoseWithCovarianceStamped, queue_size=10)

        # Odometry: Global EKF의 odom0 입력용 (회전 무관, 위치만 보정)
        self.odom_pub = rospy.Publisher('/gps_odom', Odometry, queue_size=10)
        
        # NavSatFix: hdl_graph_slam 등 표준 패키지 입력용 (추가됨)
        self.navsat_pub = rospy.Publisher('/gps/navsat', NavSatFix, queue_size=10)

    def gps_callback(self, msg):
        # 1. 현재 위경도를 UTM 좌표로 변환
        easting, northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)

        # 2. Offset 적용 (Local X, Y 계산)
        if self.use_map_anchor:
            local_x = easting - self.anchor_east
            local_y = northing - self.anchor_north
        else:
            local_x = easting - msg.eastOffset
            local_y = northing - msg.northOffset

        stamp = msg.header.stamp

        # ── PoseWithCovarianceStamped (기존 호환용) ─────────────────────
        pose_stamped = PoseWithCovarianceStamped()
        pose_stamped.header.stamp = stamp
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.pose.position.x = local_x
        pose_stamped.pose.pose.position.y = local_y
        pose_stamped.pose.pose.position.z = msg.altitude
        pose_stamped.pose.pose.orientation.w = 1.0
        
        cov = [0.0] * 36
        cov[0]  = 0.1   # X
        cov[7]  = 0.1   # Y
        cov[14] = 1e6   # Z (미사용)
        cov[21] = 1e6   # Roll (미사용)
        cov[28] = 1e6   # Pitch (미사용)
        cov[35] = 1e6   # Yaw (미사용)
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
        cov_odom[0]  = 5.0    # X
        cov_odom[7]  = 5.0    # Y
        cov_odom[14] = 1e6    # Z (무시)
        cov_odom[21] = 1e6    # Roll (무시)
        cov_odom[28] = 1e6    # Pitch (무시)
        cov_odom[35] = 1e6    # Yaw
        gps_odom.pose.covariance = cov_odom
        self.odom_pub.publish(gps_odom)

        # ── NavSatFix (hdl_graph_slam 입력용 추가) ──────────────────────
        navsat_msg = NavSatFix()
        navsat_msg.header = msg.header
        navsat_msg.header.frame_id = "gps_link" # 또는 base_link
        navsat_msg.latitude = msg.latitude
        navsat_msg.longitude = msg.longitude
        navsat_msg.altitude = msg.altitude
        
        # Status 설정
        navsat_msg.status.status = NavSatStatus.STATUS_FIX
        navsat_msg.status.service = NavSatStatus.SERVICE_GPS
        
        # Covariance (임의로 작은 값 설정, 필요시 튜닝)
        navsat_msg.position_covariance = [0.0] * 9
        navsat_msg.position_covariance[0] = 0.1 # X
        navsat_msg.position_covariance[4] = 0.1 # Y
        navsat_msg.position_covariance[8] = 0.5 # Z
        navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.navsat_pub.publish(navsat_msg)

if __name__ == '__main__':
    try:
        gps_node = GPSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
