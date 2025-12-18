
import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import numpy as np
import math as m
from scipy.spatial import Delaunay
from scipy.spatial.distance import cdist
from scipy.interpolate import UnivariateSpline, CubicSpline
from scipy.spatial.transform import Rotation

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        # Option
        self.waypoints_distance_thresold = 6.0 # waypoint가 서로 이어질 수 있는 최대 거리(float)
        self.waypoints_thresold = 5 # 두번째와 세번째 waypoint가 고정되는 waypoints의 최소 개수(int)
        self.cone_distance_threshold = 10.0 # 중점이 생길 수 있는 콘 사이의 최대거리
        # Subscribers
        self.yellow_sub = self.create_subscription(PointStamped, 'yellow_odom', self.yellow_callback, 10)
        self.blue_sub = self.create_subscription(PointStamped, 'blue_odom', self.blue_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odometry/navsat", self.odom_callback, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, 'del_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'del_array', 10)
        
        # Frame id
        self.frame_id = 'odom'

        # Initialize lists to store points
        self.yellow_points = []
        self.blue_points = []

        # Set flag
        self.flag_1 = False # starting point flag
        self.flag_2 = False # second point flag
        self.flag_3 = False # last point flage
        '''
        # Current state
        try:
            self.input_value  = sys.argv[1]
            if self.input_value == 1: # 콘이 잘못 잡힌경우
                self.f = open("cone_pose.txt", 'r', encoding='utf-8')
                lines = self.f.readlines()
                for i in lines:
                    cone,x,y = i.split(",")
                    if cone == 'yellow':
                        self.yellow_points.append(np.array([x,y]))
                    else:
                        self.blue_points.append(np.array([x,y]))
            self.f = open("cone_pose.txt", 'w', encoding='utf-8') 

        except IndexError:
            self.f = open("cone_pose.txt", 'w', encoding='utf-8')        
            pass
        '''
        # Timer to periodically publish path and markers
        self.timer = self.create_timer(0.25, self.path_create)

    def yellow_callback(self, msg):
        y_point = np.array([msg.point.x, msg.point.y])
        for point in self.blue_points: # 이미 감지된 파란색 콘과 겹칠 경우 제외
            d = np.hypot(y_point[0] - point[0],y_point[1] - point[1])
            if d < 0.3: 
                # 정확한 배열 비교를 위해 np.array_equal 사용
                index_to_remove = None
                for idx, blue_point in enumerate(self.blue_points):
                    if np.array_equal(blue_point, point):
                        index_to_remove = idx
                        break
                if index_to_remove is not None:
                    self.blue_points.pop(index_to_remove)
                    
                first_blue_distances = cdist([[msg.point.x, msg.point.y]], self.blue_points)
                first_yellow_distances = cdist([[msg.point.x, msg.point.y]], self.yellow_points)
                first_blue_closest_idx = np.argmin(first_blue_distances)
                first_yellow_closest_idx = np.argmin(first_yellow_distances)
                first_closest_blue_cone = self.blue_points[first_blue_closest_idx]
                first_closest_yellow_cone = self.yellow_points[first_yellow_closest_idx]

                copied_blue_points = self.blue_points
                copied_yellow_points = self.yellow_points
                copied_blue_points = np.delete(copied_blue_points, first_blue_closest_idx, axis=0)
                copied_yellow_points = np.delete(copied_yellow_points, first_yellow_closest_idx, axis=0)

                second_blue_distances = cdist([[msg.point.x, msg.point.y]], copied_blue_points)
                second_yellow_distances = cdist([[msg.point.x, msg.point.y]], copied_yellow_points)
                second_blue_closest_idx = np.argmin(second_blue_distances)
                second_yellow_closest_idx = np.argmin(second_yellow_distances)
                second_closest_blue_cone = self.blue_points[second_blue_closest_idx]
                second_closest_yellow_cone = self.yellow_points[second_yellow_closest_idx]
                blue_p2l_distance = self.point_to_line_distance(first_closest_blue_cone, second_closest_blue_cone, y_point)
                yellow_p2l_distance = self.point_to_line_distance(first_closest_yellow_cone, second_closest_yellow_cone, y_point)
                
                print("blue_distance:", blue_p2l_distance)
                print("yellow_distance:", yellow_p2l_distance)
                if blue_p2l_distance < yellow_p2l_distance:
                    self.blue_points.append(y_point)
                else:
                    self.yellow_points.append(y_point)
                print("섞임")
                return
            
        # self.f.write(f"yellow, {msg.point.x}, {msg.point.y}\n")
        self.yellow_points.append(y_point)

    def blue_callback(self, msg):
        b_point = np.array([msg.point.x, msg.point.y])
        for point in self.yellow_points: # 이미 감지된 노란색 콘과 겹칠 경우 제외
            d = np.hypot(b_point[0] - point[0],b_point[1] - point[1])
            if d < 0.3:   
                # 정확한 배열 비교를 위해 np.array_equal 사용
                index_to_remove = None
                for idx, yellow_point in enumerate(self.yellow_points):
                    if np.array_equal(yellow_point, point):
                        index_to_remove = idx
                        break
                if index_to_remove is not None:
                    self.yellow_points.pop(index_to_remove)
                
                # 겹친 콘과 가장 가까운 콘 구하기(파란콘, 노란콘 각각에 대하여)
                first_blue_distances = cdist([[msg.point.x, msg.point.y]], self.blue_points)
                first_yellow_distances = cdist([[msg.point.x, msg.point.y]], self.yellow_points)
                first_blue_closest_idx = np.argmin(first_blue_distances)
                first_yellow_closest_idx = np.argmin(first_yellow_distances)
                first_closest_blue_cone = self.blue_points[first_blue_closest_idx]
                first_closest_yellow_cone = self.yellow_points[first_yellow_closest_idx]

                copied_blue_points = self.blue_points
                copied_yellow_points = self.yellow_points
                copied_blue_points = np.delete(copied_blue_points, first_blue_closest_idx, axis=0)
                copied_yellow_points = np.delete(copied_yellow_points, first_yellow_closest_idx, axis=0)
                
                # 겹친 콘과 두번쨰로 가까운 콘 구하기(파란콘, 노란콘 각각에 대하여)
                second_blue_distances = cdist([[msg.point.x, msg.point.y]], copied_blue_points)
                second_yellow_distances = cdist([[msg.point.x, msg.point.y]], copied_yellow_points)
                second_blue_closest_idx = np.argmin(second_blue_distances)
                second_yellow_closest_idx = np.argmin(second_yellow_distances)
                second_closest_blue_cone = self.blue_points[second_blue_closest_idx]
                second_closest_yellow_cone = self.yellow_points[second_yellow_closest_idx]
                blue_p2l_distance = self.point_to_line_distance(first_closest_blue_cone, second_closest_blue_cone, b_point)
                yellow_p2l_distance = self.point_to_line_distance(first_closest_yellow_cone, second_closest_yellow_cone, b_point)
                
                print("blue_distance:", blue_p2l_distance)
                print("yellow_distance:", yellow_p2l_distance)
                if blue_p2l_distance < yellow_p2l_distance:
                    self.blue_points.append(b_point)
                else:
                    self.yellow_points.append(b_point)
                print("섞임")
            
        # self.f.write(f"blue, {msg.point.x}, {msg.point.y}\n")
        self.blue_points.append(b_point)
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x 
        self.current_y = msg.pose.pose.position.y
        if not self.flag_1: # waypoint 시작점 정하기 
            self.first_point = [self.current_x,self.current_y]
            self.flag_1 = True

    def path_create(self):
        if len(self.yellow_points) < 2 or len(self.blue_points) < 2:
            return
        
        # Combine yellow and blue points
        all_points = np.array(self.yellow_points + self.blue_points)
        
        # Perform Delaunay triangulation
        tri = Delaunay(all_points)
        triangles = tri.simplices
        pairs = self.find_edge_pairs(self.yellow_points, self.blue_points)
        midpoints = self.calculate_midpoints(all_points, triangles, pairs)
        waypoints = self.rearrange_midpoints(self.first_point, midpoints)
        spline_x, spline_y = self.spline_interpolation(waypoints, s=0.1)

        # Publish path
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        dx = np.diff(spline_x)
        dy = np.diff(spline_y)
        if len(spline_x) > 2:
            dx = np.append(dx,dx[-1])
            dy = np.append(dy,dy[-1])

        for x, y, dx, dy in zip(spline_x, spline_y, dx, dy):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = -1.0
            yaw = np.arctan2(dy, dx)
            q = Rotation.from_euler('z', [yaw], degrees=False)
            quat = q.as_quat()[0]

            pose.pose.orientation.x = float(quat[0])
            pose.pose.orientation.y = float(quat[1])
            pose.pose.orientation.z = float(quat[2])
            pose.pose.orientation.w = float(quat[3])
            path.poses.append(pose)
        
        self.path_pub.publish(path)

        # Create MarkerArray for Delaunay triangles
        marker_array = MarkerArray()
        self.add_triangles_to_marker_array(marker_array, all_points, triangles)
        self.add_points_to_marker_array(marker_array, all_points, 'points', 1.0, 0.0, 0.0)
        self.add_points_to_marker_array(marker_array, midpoints, 'midpoints', 1.0, 1.0, 1.0)
        self.marker_pub.publish(marker_array)

    def point_to_line_distance(self, first_cone, second_cone, error_cone):
        """
        두 점(first_cone, second_cone)으로 정의된 직선과 세 번째 점(point) 사이의 거리를 계산합니다.

        :param first_cone: numpy.ndarray 형태의 첫 번째 점 (x1, y1)
        :param second_cone: numpy.ndarray 형태의 두 번째 점 (x2, y2)
        :param point: numpy.ndarray 형태의 세 번째 점 (x, y)
        :return: 세 번째 점과 직선 사이의 거리 (float)
        """
        
        # 직선의 두 점 사이의 벡터를 구합니다
        line_vec = second_cone - first_cone
        
        # 첫 번째 점과 주어진 점 사이의 벡터를 구합니다
        point_vec = error_cone - first_cone
        
        # 직선 벡터에 대한 점 벡터의 투영을 구합니다
        line_len = np.linalg.norm(line_vec)
        line_unitvec = line_vec / line_len
        projection_length = np.dot(point_vec, line_unitvec)
        
        # 투영 벡터를 구합니다
        projection_vec = projection_length * line_unitvec
        
        # 직선과 점 사이의 거리 벡터를 구합니다
        distance_vec = point_vec - projection_vec
        
        # 거리 벡터의 크기(즉, 거리)를 반환합니다
        distance = np.linalg.norm(distance_vec)
        
        return distance
    
    # 왼쪽 콘과 오른쪽 콘의 인덱스 쌍을 찾기 위한 함수
    def find_edge_pairs(self, cones1, cones2):
        indices1 = range(len(cones1))
        indices2 = range(len(cones1), len(cones1) + len(cones2))
        pairs = [(i, j) for i in indices1 for j in indices2]
        return pairs
        
    # Calculate midpoints
    def calculate_midpoints(self, cones, simplex_indices, pairs):
        midpoints = []
        for simplex in simplex_indices:
            for i in range(len(simplex)):
                for j in range(i + 1, len(simplex)):
                    if (simplex[i], simplex[j]) in pairs or (simplex[j], simplex[i]) in pairs:
                        p1 = cones[simplex[i]]
                        p2 = cones[simplex[j]]
                        distance = np.hypot(p1[0] - p2[0], p1[1] - p2[1])
                        if distance < self.cone_distance_threshold:
                            midpoint = (p1 + p2) / 2
                            midpoints.append(midpoint)
        midpoints = np.unique(midpoints, axis=0)     
        return midpoints
    
    # 현재 점에서 가장 가까운 점을 찾는 함수
    def rearrange_midpoints(self, starting_point, points):
        remaining_points = points.copy()
        waypoints = [starting_point]
        if not self.flag_3: # watpoints의 마지막 점이 결정되지 않았을 경우
            if self.flag_2: # watpoints의 두번때, 세번째 점이 결정되었을 경우
                remaining_points = remaining_points[~np.all(remaining_points == self.first_point, axis=1)]
                remaining_points = remaining_points[~np.all(remaining_points == self.second_point, axis=1)]
                front_points = [starting_point, self.second_point]
                current_point = self.third_point
                while len(remaining_points) > 0:
                    distances = cdist([current_point], remaining_points)
                    closest_idx = np.argmin(distances)
                    if distances[0][closest_idx] < self.waypoints_distance_thresold:
                        closest_point = remaining_points[closest_idx] 
                        waypoints.append(list(closest_point))
                        current_point = closest_point
                    remaining_points = np.delete(remaining_points, closest_idx, axis=0)
                front_points.extend(waypoints)
                start_end_distance = np.hypot(self.first_point[0] - waypoints[-1][0], self.first_point[1] - waypoints[-1][1])

                if start_end_distance < 1.0: # 시작점과 waypoints의 마지막 점 사이의 거리가 1.0 미만이면 path를 완성으로 판단
                   waypoints.append(self.first_point)
                   self.final_waypoints = waypoints
                   self.flag_3 = True

            else: # watpoints의 두번째 점이 결정되지 않았을 경우
                current_point = starting_point
                while len(remaining_points) > 0:
                    distances = cdist([current_point], remaining_points)
                    closest_idx = np.argmin(distances)
                    if distances[0][closest_idx] < self.waypoints_distance_thresold:
                        closest_point = remaining_points[closest_idx] 
                        waypoints.append(list(closest_point))
                        current_point = closest_point
                    remaining_points = np.delete(remaining_points, closest_idx, axis=0)
                    
                if len(waypoints) > self.waypoints_thresold + 2: # waypoint의 개수가 일정 개수 이상일 경우
                    self.second_point = waypoints[1]
                    self.third_point = waypoints[2]
                    self.flag_2 = True

        else:
            waypoints = self.final_waypoints # 이후의 waypoints에는 완성된 path를 계속 넣어줌

        return np.array(waypoints)
    
    # UnivariateSpline을 사용하여 경로를 부드럽게 연결하는 함수
    def spline_interpolation(self, points, s=1, k=3):
        if len(points) < 2:
            return points[:, 0], points[:, 1]
        
        # x, y 좌표 분리
        x = points[:, 0]
        y = points[:, 1]
        
        # # 스플라인 보간법 적용
        # us_x = UnivariateSpline(range(len(x)), x, s=s, k=k)
        # us_y = UnivariateSpline(range(len(y)), y, s=s, k=k)
        us_x = CubicSpline(range(len(x)), x)
        us_y = CubicSpline(range(len(y)), y)

        # 보간할 구간 설정
        xs = np.linspace(0, len(x) - 1, len(x)*10)
        ys = us_x(xs)
        zs = us_y(xs)
        
        return ys, zs      

    def add_triangles_to_marker_array(self, marker_array, points, triangles):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'triangles'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.scale.x = 0.03

        for simplex in triangles:
            for i in range(3):
                p1 = points[simplex[i]]
                p2 = points[simplex[(i + 1) % 3]]
                marker.points.append(self.create_marker_point(p1))
                marker.points.append(self.create_marker_point(p2))

        marker_array.markers.append(marker)

    def add_points_to_marker_array(self, marker_array, points, ns, r, g, b):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1

        for point in points:
            marker.points.append(self.create_marker_point(point))

        marker_array.markers.append(marker)

    def create_marker_point(self, point):
        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = -1.0  # Fixed Z-axis value
        return p

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
