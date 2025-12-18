#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from erp42_msgs.msg import ControlMessage
from visualization_msgs.msg import Marker
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree
from scipy.interpolate import splprep, splev
from tf_transformations import quaternion_from_euler

class ConeTracker(Node):
    def __init__(self):
        super().__init__('cone_tracker_node')
        # 상태 변수 및 파라미터 초기화
        self.x, self.y, self.yaw, self.v = 0.0, 0.0, 0.0, 0.0
        self.L = 2.24  # 차량 휠베이스
        self.k_v = 0.5  # 스탠리 제어 파라미터
        self.cx, self.cy, self.cyaw = [], [], [] # 경로 좌표

        # ROS2 Subscriptions
        self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/cone_pose_map', self.cone_callback, 10)

        # ROS2 Publishers
        self.path_pub = self.create_publisher(Path, '/center_path', 10)
        self.left_pub = self.create_publisher(Marker, '/cones_left_line', 10)
        self.right_pub = self.create_publisher(Marker, '/cones_right_line', 10)
        self.cmd_pub = self.create_publisher(ControlMessage, '/cmd_msg', 10)

        # 타이머 (제어 루프)
        self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg: Odometry):
        """차량의 위치 및 속도 업데이트"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        self.yaw = R.from_quat(quat).as_euler('zyx')[0]
        self.v = msg.twist.twist.linear.x

    def cone_callback(self, msg: PoseArray):
        """콘 데이터를 처리하고 경로를 생성"""
        cones = [(p.position.x, p.position.y) for p in msg.poses]
        if len(cones) < 2: return

        cones_v = self._world_to_vehicle(cones) # 차량 좌표계로 변환
        
        # 전방 콘만 필터링 (X>0)
        forward_cones = cones_v[cones_v[:, 0] > 0.0]
        left_cones_v = forward_cones[forward_cones[:, 1] > 0.0]
        right_cones_v = forward_cones[forward_cones[:, 1] < 0.0]

        # 좌우 콘 매칭 및 중점 계산
        pairs = self._match_cones(left_cones_v, right_cones_v)
        if not pairs:
            # 콘 쌍이 없을 경우, 기존 경로를 유지
            self.get_logger().warn("Not enough cone pairs found. Maintaining previous path.")
            return
        
        midpoints_v = [((l[0]+r[0])/2, (l[1]+r[1])/2) for l, r in pairs]

        # 중점을 다시 월드 좌표계로 변환하고 경로 생성
        midpoints_w = self._vehicle_to_world(np.array(midpoints_v))
        cx_raw, cy_raw = zip(*midpoints_w) if midpoints_w else ([], [])
        
        if len(cx_raw) > 1:
            self.cx, self.cy, self.cyaw = self._spline_interpolate(list(cx_raw), list(cy_raw))
        
        self._publish_visualizations(left_cones_v, right_cones_v)
        self._publish_path()

    def _match_cones(self, left, right):
        """
        차량으로부터의 거리를 기준으로 콘들을 정렬하여 순차적으로 페어링합니다.
        이를 통해 곡선 경로에서도 더 안정적인 경로를 생성할 수 있습니다.
        """
        if left.size == 0 or right.size == 0:
            return []
        
        # x좌표(전방 거리)를 기준으로 콘들을 오름차순 정렬
        left_sorted = left[np.argsort(left[:, 0])]
        right_sorted = right[np.argsort(right[:, 0])]
        
        pairs = []
        # 정렬된 리스트에서 순서대로 콘들을 페어링
        for l_cone, r_cone in zip(left_sorted, right_sorted):
            pairs.append((l_cone, r_cone))
        
        return pairs

    def _spline_interpolate(self, cx, cy, num_points=100):
        """스플라인 보간을 통해 부드러운 경로 생성"""
        # 현재 위치를 보간에 사용할 점 목록에 추가
        points = [(self.x, self.y)] + list(zip(cx, cy))
        if len(points) < 2: return list(cx), list(cy), [self.yaw] * len(cx)
        
        unique_points = list(dict.fromkeys(points))
        x, y = zip(*unique_points)
        
        k = min(3, len(x) - 1)
        try:
            tck, _ = splprep([x, y], s=0, k=k)
            u_fine = np.linspace(0, 1, num_points)
            sx, sy = splev(u_fine, tck)
            yaw = np.arctan2(np.gradient(sy), np.gradient(sx))
        except Exception as e:
            self.get_logger().warn(f"Spline interpolation failed: {e}")
            sx, sy = x, y
            yaw = [self.yaw] * len(x)
        
        return sx.tolist(), sy.tolist(), yaw.tolist()

    def control_loop(self):
        """차량 제어 명령을 발행"""
        if not self.cx or not self.cy or not self.cyaw:
            # 경로가 없을 경우, 정지
            msg = ControlMessage()
            msg.speed = 0
            msg.gear = 2
            msg.brake = 100
            self.cmd_pub.publish(msg)
            return

        # 스탠리 컨트롤러로 조향각 계산
        target_idx = np.argmin(np.hypot(self.x - np.array(self.cx), self.y - np.array(self.cy)))
        target_yaw = self.cyaw[target_idx]
        
        yaw_error = self._normalize_angle(target_yaw - self.yaw)
        cross_track_error = math.sin(target_yaw) * (self.x - self.cx[target_idx]) - math.cos(target_yaw) * (self.y - self.cy[target_idx])
        
        steer = yaw_error + math.atan2(self.k_v * cross_track_error, self.v + 1e-5)
        steer = max(min(steer, math.radians(28)), math.radians(-28))

        # 속도 및 제어 메시지 발행
        msg = ControlMessage()
        msg.steer = int((math.degrees(-steer))*1e3)
        msg.speed = 120 # 7.0m/s
        msg.gear = 2
        msg.brake = 0
        self.cmd_pub.publish(msg)

    # --- 유틸리티 함수들 ---
    def _world_to_vehicle(self, pts):
        """월드 좌표를 차량 좌표로 변환"""
        if not pts: return np.zeros((0, 2))
        x0, y0, yaw = self.x, self.y, self.yaw
        dx = np.array([p[0] - x0 for p in pts])
        dy = np.array([p[1] - y0 for p in pts])
        cy, sy = math.cos(yaw), math.sin(yaw)
        x_v =  cy * dx + sy * dy
        y_v = -sy * dx + cy * dy
        return np.column_stack([x_v, y_v])

    def _vehicle_to_world(self, pts_v):
        """차량 좌표를 월드 좌표로 변환"""
        if pts_v.size == 0: return []
        pts_v = np.asarray(pts_v, dtype=float).reshape(-1, 2)
        x0, y0, yaw = self.x, self.y, self.yaw
        cy, sy = math.cos(yaw), math.sin(yaw)
        x_w = cy * pts_v[:, 0] - sy * pts_v[:, 1] + x0
        y_w = sy * pts_v[:, 0] + cy * pts_v[:, 1] + y0
        return list(zip(x_w.tolist(), y_w.tolist()))
        
    def _normalize_angle(self, angle):
        """각도를 -pi ~ pi 범위로 정규화"""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _publish_path(self):
        """생성된 중앙 경로 시각화"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in zip(self.cx, self.cy, self.cyaw):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x, pose.pose.position.y = x, y
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            path.poses.append(pose)
        self.path_pub.publish(path)
    
    def _publish_visualizations(self, left_cones_v, right_cones_v):
        """좌우 콘 라인 시각화"""
        left_cones_w = self._vehicle_to_world(left_cones_v)
        right_cones_w = self._vehicle_to_world(right_cones_v)

        # 왼쪽 라인
        left_marker = self._create_line_marker(left_cones_w, color_rgb=(0.0, 0.0, 1.0))
        self.left_pub.publish(left_marker)
        
        # 오른쪽 라인
        right_marker = self._create_line_marker(right_cones_w, color_rgb=(1.0, 1.0, 0.0))
        self.right_pub.publish(right_marker)

    def _create_line_marker(self, points, color_rgb):
        """Marker 메시지를 생성하는 헬퍼 함수"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.r, marker.color.g, marker.color.b = color_rgb
        marker.color.a = 1.0
        # x좌표를 기준으로 정렬하여 선을 그림
        marker.points = [Point(x=p[0], y=p[1], z=0.0) for p in sorted(points, key=lambda p: p[0])]
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = ConeTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()