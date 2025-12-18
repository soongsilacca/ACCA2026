#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from math import cos, sin, sqrt, degrees
from shapely.geometry import Point, LineString
from geometry_msgs.msg import Point as ROSPoint
from stanley import Stanley
from erp42_msgs.msg import ControlMessage

# ★ DB 연동
from DB import DB


class SpeedSupporter:
    def __init__(self, node):
        self.he_gain = node.declare_parameter("/speed_supporter/he_gain_obstacle", 70.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain_obstacle", 30.0).value
        self.he_thr  = node.declare_parameter("/speed_supporter/he_thr_obstacle", 0.001).value
        self.ce_thr  = node.declare_parameter("/speed_supporter/ce_thr_obstacle", 0.002).value  # ★ 오타 수정

    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self, value, hdr, ctr, min_value, max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)
        return res


class PID:
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain_obstacle", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain_obstacle", 0.85).value

        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0

        now = node.get_clock().now().seconds_nanoseconds()
        self.current = now[0] + (now[1] / 1e9)
        self.last    = self.current

    def PIDControl(self, speed, desired_value, min_var = 6, max_var = 14):
        now = self.node.get_clock().now().seconds_nanoseconds()
        self.current = now[0] + (now[1] / 1e9)
        dt = max(1e-3, self.current - self.last)
        self.last = self.current

        err = desired_value - speed
        self.p_err = err
        self.i_err += self.p_err * dt * (0.0 if speed == 0 else 1.0)

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(self.speed, min_var, max_var))


class Obstacle:
    def __init__(self, node: Node):
        self.node = node
# small obstacle/delivery wapoint 당기기 ##
        # Subscribers (MarkerArray → PoseArray)
        # PoseArray는 map frame 기준의 콘 위치(poses)들을 담고 있다고 가정
        self.sub_cone_pose = self.node.create_subscription(
            PoseArray, "/cone_pose_map", self.call_cone_pose, 10
        )

        # Publishers
        self.ref_path1     = self.node.create_publisher(Path, "/ref/path1", 10)      # L1
        self.ref_path2     = self.node.create_publisher(Path, "/ref/path2", 10)      # L2
        self.LocalPath_pub = self.node.create_publisher(Path, "/path/avoid_path", 10)
        self.marker_pub    = self.node.create_publisher(MarkerArray, "transformed_markers", 10)
        self.road_poly     = self.node.create_publisher(Marker, "visualization_marker", 10)
        
        # util function 모음
        # 버퍼 폭 (m)
        self.buffer_m = 0.45
        self.small_hgain = 0.5
        self.small_cgain = 1.3 # 줄여야함
        print(f"{self.small_cgain}")
        print(f"{self.small_cgain}")
        print(f"{self.small_cgain}")
        print(f"{self.small_cgain}")
        print(f"{self.small_cgain}")
        print(f"{self.small_cgain}")
        self.big_hgain = 1.0
        self.big_cgain = 1.3
        self.small_end_dindex = 100
        self.big_end_dindex = 50 ## mission finish after lane change paramete 50 ( 이동하고 확인 )
        self.small_thr = 3.7
        self.big_thr = 5.5
        
        
        # Odom / pose (외부에서 주입되는 odometry 객체의 필드: x, y, yaw, v)
        self.odometry = None
        self.odom_pose = np.array([0.0, 0.0])
        self.odom_orientation = [0.0, 0.0, 0.0, 1.0]

        # Obstacles
        self.obs = np.empty((0, 2))  # map frame (x, y)
        self.num1_obs = []
        self.num2_obs = []
        self.obs_vector = None
        self.to_num = None
        self.standard_idx = 0
        self.flag_first_change = True
        self.buffer = 0

        # Paths (from DB)
        self.ref_path_points1 = None  # np Nx2
        self.ref_path_points2 = None  # np Nx2

        # Local path
        self.local_points = None  # list of (x, y)
        self.local_x = []
        self.local_y = []
        self.local_yaw = []

        self.st  = Stanley()
        self.pid = PID(node)
        self.ss  = SpeedSupporter(node)

        self.code_start = True
        self.target_idx = 0
        self.state = None

        # ==== DB에서 4개 경로 읽기 (path_id/idx 모두 안 씀) ====
        # 파일 위치는 DB.py 내부 규칙(/home/libok/db_file/<name>)에 따름
        self.db_small_l1 = DB("small_l1.db")
        self.db_small_l2 = DB("small_l2.db")
        self.db_big_l1   = DB("big_l1.db")
        self.db_big_l2   = DB("big_l2.db")

        def _rows_to_arrays_xyz(rows):
            # read_db_n("Path","x","y","yaw") -> [(x,y,yaw), ...]
            if not rows:
                return np.array([]), np.array([]), np.array([])
            xs = [float(r[0]) for r in rows]
            ys = [float(r[1]) for r in rows]
            yaws = [float(r[2]) for r in rows]
            return np.array(xs), np.array(ys), np.array(yaws)

        small_l1_rows = self.db_small_l1.read_db_n("Path", "x", "y", "yaw")
        small_l2_rows = self.db_small_l2.read_db_n("Path", "x", "y", "yaw")
        big_l1_rows   = self.db_big_l1.read_db_n("Path", "x", "y", "yaw")
        big_l2_rows   = self.db_big_l2.read_db_n("Path", "x", "y", "yaw")

        self.cx_small_l1, self.cy_small_l1, self.cyaw_small_l1 = _rows_to_arrays_xyz(small_l1_rows)
        self.cx_small_l2, self.cy_small_l2, self.cyaw_small_l2 = _rows_to_arrays_xyz(small_l2_rows)
        self.cx_big_l1,   self.cy_big_l1,   self.cyaw_big_l1   = _rows_to_arrays_xyz(big_l1_rows)
        self.cx_big_l2,   self.cy_big_l2,   self.cyaw_big_l2   = _rows_to_arrays_xyz(big_l2_rows)

        # 작은/큰 선택용(L2 첫 점)
        self._small_l2_first = (float(self.cx_small_l2[0]), float(self.cy_small_l2[0])) if len(self.cx_small_l2) > 0 else None
        self._big_l2_first   = (float(self.cx_big_l2[0]),   float(self.cy_big_l2[0]))   if len(self.cx_big_l2) > 0   else None


    # ===== PoseArray 콜백 =====
    def call_cone_pose(self, msg: PoseArray):
        # PoseArray는 이미 map frame 기준 → 변환 불필요. 항상 갱신
        observed_points = [[p.position.x, p.position.y] for p in msg.poses]
        self.obs = np.array(observed_points) if observed_points else np.empty((0, 2))
        self.node.get_logger().debug(f"[cone_pose] received={len(observed_points)} cones")

    # ====== DB 경로를 바로 Path로 퍼블리시 ======
    def _publish_path_from_arrays(self, cx, cy, num):
        path = Path()
        path.header = Header()
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.header.frame_id = "map"

        for i in range(max(0, len(cx) - 1)):
            x = float(cx[i]); y = float(cy[i])
            nx = float(cx[i + 1]); ny = float(cy[i + 1])
            yaw = np.arctan2(ny - y, nx - x)
            q = quaternion_from_euler(0, 0, yaw)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)

        pts = np.vstack((cx, cy)).T if len(cx) > 0 else np.empty((0, 2))
        if num == 1:
            self.ref_path1.publish(path)
            self.ref_path_points1 = pts
        elif num == 2:
            self.ref_path2.publish(path)
            self.ref_path_points2 = pts

    def obs_marker(self):
        marker_array = MarkerArray()
        for idx, point in enumerate(self.num1_obs):
            mk = Marker()
            mk.header.frame_id = "map"
            mk.header.stamp = self.node.get_clock().now().to_msg()
            mk.ns = "num1_obs_markers"
            mk.id = idx
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position.x = point[0]
            mk.pose.position.y = point[1]
            mk.pose.position.z = 0.0
            mk.scale.x = mk.scale.y = mk.scale.z = 0.5
            mk.color.a = 1.0; mk.color.r = 0.0; mk.color.g = 1.0; mk.color.b = 0.0
            marker_array.markers.append(mk)

        for idx, point in enumerate(self.num2_obs):
            mk = Marker()
            mk.header.frame_id = "map"
            mk.header.stamp = self.node.get_clock().now().to_msg()
            mk.ns = "num2_obs_markers"
            mk.id = len(self.num1_obs) + idx
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position.x = point[0]
            mk.pose.position.y = point[1]
            mk.pose.position.z = 0.0
            mk.scale.x = mk.scale.y = mk.scale.z = 0.5
            mk.color.a = 1.0; mk.color.r = 1.0; mk.color.g = 0.0; mk.color.b = 0.0
            marker_array.markers.append(mk)

        self.marker_pub.publish(marker_array)

    def organize_obstacle_lists(self):
        current_position = np.array([self.odometry.x, self.odometry.y])

        # num1: 가장 가까운 num2와의 거리가 최소가 되는 한 점만 남김 (원래 로직 유지)
        if self.num1_obs:
            distances_num1 = [np.linalg.norm(np.array(p) - current_position) for p in self.num1_obs]
            best_point = None
            best_distance = -1
            best_closest_distance = float("inf")
            for i, point1 in enumerate(self.num1_obs):
                distance_from_current = distances_num1[i]
                if self.num2_obs:
                    closest_distance = min(np.linalg.norm(np.array(point1) - np.array(point2)) for point2 in self.num2_obs)
                else:
                    closest_distance = float("inf")
                if (closest_distance < best_closest_distance) or \
                   (closest_distance == best_closest_distance and distance_from_current > best_distance):
                    best_point = point1
                    best_distance = distance_from_current
                    best_closest_distance = closest_distance
            if best_point is not None:
                self.num1_obs = [best_point]

        # num2: 현재 위치와의 거리 기준으로 3개까지 남김
        if self.num2_obs:
            distances_num2 = [np.linalg.norm(np.array(p) - current_position) for p in self.num2_obs]
            closest_indices = np.argsort(distances_num2)[:3]
            self.num2_obs = [self.num2_obs[i] for i in closest_indices]

    def publish_polygon(self, points, ns, r, g, b, a):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.color.a = a; marker.color.r = r; marker.color.g = g; marker.color.b = b

        for point in points:
            p = ROSPoint(); p.x = float(point[0]); p.y = float(point[1]); p.z = 0.0
            marker.points.append(p)
        first = ROSPoint(); first.x = float(points[0][0]); first.y = float(points[0][1]); first.z = 0.0
        marker.points.append(first)
        self.road_poly.publish(marker)

    def publish_local_path(self, points):
        # points: list of (x, y)
        if not points or len(points) < 2:
            return

        local_x = [float(p[0]) for p in points]
        local_y = [float(p[1]) for p in points]

        local_x = np.asarray(local_x)
        local_y = np.asarray(local_y)

        dx = np.diff(local_x); dy = np.diff(local_y)
        distances = np.sqrt(dx**2 + dy**2)
        total_distance = np.cumsum(distances)
        total_distance = np.insert(total_distance, 0, 0.0)

        if total_distance[-1] <= 0.0:
            return

        interp_distances = np.arange(0.0, total_distance[-1], 0.1)
        if len(interp_distances) < 2:
            return

        interp_x = np.interp(interp_distances, total_distance, local_x)
        interp_y = np.interp(interp_distances, total_distance, local_y)

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.node.get_clock().now().to_msg()

        local_yaw = []
        for i in range(len(interp_x) - 1):
            x = interp_x[i]; y = interp_y[i]
            nx = interp_x[i + 1]; ny = interp_y[i + 1]
            yaw = np.arctan2(ny - y, nx - x); local_yaw.append(yaw)
            q = quaternion_from_euler(0, 0, yaw)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = -1.0
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)

        self.LocalPath_pub.publish(path)
        self.local_x = interp_x.tolist()
        self.local_y = interp_y.tolist()
        self.local_yaw = local_yaw

    def line_change(self):
        # 참고: num1은 보조 기준(옆 차선 콘 등), num2는 주행 차선 콘 등으로 가정
        if len(self.num1_obs) > 0:
            for obs_x, obs_y in self.num1_obs:
                _, _, yaw = euler_from_quaternion(self.odom_orientation)
                vehicle_direction = np.array([cos(yaw), sin(yaw)])
                self.obs_vector = np.array([obs_x - self.odometry.x, obs_y - self.odometry.y])

        # 최소 1개만 있어도 회피 고려 + 전방 필터
        if len(self.num2_obs) >= 1:
            _, _, yaw = euler_from_quaternion(self.odom_orientation)
            vehicle_direction = np.array([cos(yaw), sin(yaw)])

            front_obs = []
            for obs_x, obs_y in self.num2_obs:
                obs_vector = np.array([obs_x - self.odometry.x, obs_y - self.odometry.y])
                obs_length = sqrt((obs_x - self.odometry.x)**2 + (obs_y - self.odometry.y)**2)
                if np.dot(obs_vector, vehicle_direction) > 0:  # 전방만
                    front_obs.append((obs_length, obs_x, obs_y))

            if front_obs:
                front_obs.sort(key=lambda t: t[0])
                nearest_len, obs_x, obs_y = front_obs[0]
                self.node.get_logger().debug(f"[line_change] nearest front cone dist={nearest_len:.2f} m")
                thr = self.small_thr if self.state == "small" else self.big_thr # 필요시 2.0~5.0 조정
                if nearest_len <= thr:
                    mode = "긴급회피" if len(self.num1_obs) < 1 else "회피"
                    self.node.get_logger().info(f"[line_change] {mode} triggered at {nearest_len:.2f} m")
                    self.to_num = 1
                    if self.flag_first_change:
                        self.standard_idx = self.target_idx
                        self.buffer += 1
                        if self.buffer >= 5:
                            self.flag_first_change = False
                    self.local_points = self.ref_path_points1.tolist() if self.ref_path_points1 is not None else None
                    if self.local_points is not None:
                        self.publish_local_path(self.local_points)
                    return

        # 기본은 라인 2 유지
        if self.to_num is None and self.ref_path_points2 is not None:
            self.local_points = self.ref_path_points2.tolist()
        self.to_num = 2
        if self.local_points is not None:
            self.publish_local_path(self.local_points)

    # ====== DB 경로 기반 버퍼(±buffer_m m)로 장애물 포함 판정 ======
    def _build_buffer_polygon(self, ref_points_np, buf_m):
        if ref_points_np is None or len(ref_points_np) < 2:
            return None, []
        line = LineString(ref_points_np.tolist())
        poly = line.buffer(buf_m, cap_style=2)
        exterior = list(poly.exterior.coords)
        return poly, exterior

    def check_obstacle(self):
        if self.ref_path_points1 is None or self.ref_path_points2 is None:
            return
        poly1, ext1 = self._build_buffer_polygon(self.ref_path_points1, self.buffer_m)
        poly2, ext2 = self._build_buffer_polygon(self.ref_path_points2, self.buffer_m)
        if poly1 is None or poly2 is None:
            return

        self.publish_polygon(ext1, "line1_buf", 0.0, 1.0, 0.0, 1.0)
        self.publish_polygon(ext2, "line2_buf", 1.0, 0.0, 0.0, 1.0)

        self.num1_obs.clear()
        self.num2_obs.clear()

        # contains는 경계(touches)에서 False → 작은 buffer로 여유 허용
        bpoly1 = poly1.buffer(0.05)
        bpoly2 = poly2.buffer(0.05)

        for obs_point in self.obs:
            pt = Point(obs_point)
            if bpoly1.intersects(pt):  # contains/touches 모두 커버
                self.num1_obs.append((pt.x, pt.y))
            if bpoly2.intersects(pt):
                self.num2_obs.append((pt.x, pt.y))

        self.node.get_logger().debug(f"[check_obstacle] num1={len(self.num1_obs)}, num2={len(self.num2_obs)}")

    # ====== 제어 루프 ======
    def control_obstacle(self, odometry, path):
        self.odometry = odometry
        self.timer_callback()
        msg = ControlMessage()

        if len(self.local_x) != 0:
            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                odometry, self.local_x, self.local_y, self.local_yaw, h_gain=0.5, c_gain=0.3
            )
            if self.state == "big":
                target_speed = 12.0
                adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=6, max_value=14)
                speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed)
            else:
                target_speed = 5.0
                adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=4, max_value=6)
                print(f"{adapted_speed}")
                speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed, min_var = 4 , max_var = 6)

        else:
            self.h_gain = self.small_hgain if self.state == "small" else self.big_hgain
            self.c_gain = self.small_cgain if self.state == "small" else self.big_cgain
            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                odometry, path.cx, path.cy, path.cyaw, h_gain=self.h_gain, c_gain=self.c_gain
            )
            if self.state == "big":
                target_speed = 12.0
                adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=6, max_value=14)
                speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed)
            else:
                target_speed = 5.0
                adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=4, max_value=6)
                print(f"{adapted_speed}")
                speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed, min_var = 4, max_var = 6)

        msg.speed = int(speed) * 10
        msg.steer = int(degrees((-1) * steer)*1e3)
        msg.gear = 2
        end_dindex = self.small_end_dindex if self.state == "small" else self.big_end_dindex

        # 로컬 경로 끝에 가까워지면 초기화
        if self.local_points is not None and self.target_idx >= len(self.local_points) - 10:
            self.to_num = None
            self.standard_idx = 0
            self.buffer = 0
            self.code_start = False
            self.num1_obs = []
            self.num2_obs = []
            return msg, True
        elif self.flag_first_change == False and self.target_idx >= self.standard_idx + end_dindex:
            return msg, True
        else:
            return msg, False

    def timer_callback(self):
        if not self.code_start:
            self.code_start = True

        self.odom_pose = np.array([self.odometry.x, self.odometry.y])
        q = quaternion_from_euler(0, 0, self.odometry.yaw)
        self.odom_orientation = [q[0], q[1], q[2], q[3]]

        # 작은/큰 선택 (L2 첫 점과의 거리)
        if self._small_l2_first is None or self._big_l2_first is None:
            self.node.get_logger().warn("[Obstacle] L2 firstpoints not ready; skip.")
            return

        px, py = float(self.odometry.x), float(self.odometry.y)
        d_small = (px - self._small_l2_first[0])**2 + (py - self._small_l2_first[1])**2
        d_big   = (px - self._big_l2_first[0])**2   + (py - self._big_l2_first[1])**2
        use_small = d_small <= d_big

        # 선택 세트 퍼블리시 (DB 배열 그대로 사용)
        if use_small:
            self.state = "small"
            self._publish_path_from_arrays(self.cx_small_l1, self.cy_small_l1, num=1)
            self._publish_path_from_arrays(self.cx_small_l2, self.cy_small_l2, num=2)
        else:
            self.state = "big"
            self._publish_path_from_arrays(self.cx_big_l1, self.cy_big_l1, num=1)
            self._publish_path_from_arrays(self.cx_big_l2, self.cy_big_l2, num=2)

        # 버퍼 기반 장애물 판정 → 라인 변경 로직
        self.check_obstacle()
        self.organize_obstacle_lists()
        self.obs_marker()
        self.line_change()
