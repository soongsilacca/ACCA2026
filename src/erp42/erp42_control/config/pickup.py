import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
import math
from rclpy.qos import qos_profile_system_default
import os
import time

from mission.lanelet_map_loader import LaneletMap, get_osm_path_from_yaml
from mission.stanley import Stanley
from mission.state_machine import GetOdometry, euler_from_quaternion
from erp42_msgs.msg import SerialFeedBack, ControlMessage


class PickupDeliveryNode(Node):
    def __init__(self):
        super().__init__("pickup_node")

        # Odom 데이터를 구독
        self.odom_sub = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10
        )
        self.create_subscription(
            SerialFeedBack,
            "erp42_feedback",
            self.callback_erp,
            qos_profile=qos_profile_system_default,
        )

        # mission, pickup_start, pickup_goal, pickup_path 토픽 퍼블리셔 생성
        self.mission_pub = self.create_publisher(String, "/mission", 10)
        self.pickup_start_pub = self.create_publisher(PointStamped, "/pickup_start", 10)
        self.pickup_goal_pub = self.create_publisher(PointStamped, "/pickup_goal", 10)
        self.pickup_path_pub = self.create_publisher(Path, "/pickup_path", 10)

        # LaneletMap 객체 생성 및 OSM 파일 로드
        self.lanelet_map = LaneletMap()
        osm_path = get_osm_path_from_yaml(
            "/home/hovin/new_hovin_ws/src/mission/config/map.yaml"
        )
        if not self.lanelet_map.load_osm(osm_path):
            self.get_logger().error("Failed to load OSM map")
            self.destroy_node()
            return

        # Path 퍼블리시를 지속적으로 수행하기 위한 타이머 설정
        self.path_timer = self.create_timer(0.1, self.publish_path)

        # 시작점과 목표점 설정
        self.pickup_start = None
        self.pickup_goal = PointStamped()
        self.pickup_goal.header.frame_id = "map"
        self.pickup_goal.point.x = 36.4341
        self.pickup_goal.point.y = 63.5036
        self.pickup_goal.point.z = 0.0

        # x, y, yaw 리스트 초기화
        self.x_list = []
        self.y_list = []
        self.yaw_list = []

        self.target_idx = 0
        self.pickup_finished = False
        self.path_published_once = False  # 리스트를 한 번만 출력하기 위한 플래그

        self.st = Stanley()
        self.go = GetOdometry(self, "/localization/kinematic_state")

        self.data_saved = False

        self.sign_data_sub = self.create_subscription(
            BoundingBoxes, "bounding_boxes", self.sign_data_callback, 10
        )

        self.file_path = "/home/hovin/new_hovin_ws/src/mission/resource/pickup_sign.txt"
        self.directory_path = "/home/hovin/new_hovin_ws/src/mission/resource/"

        self.get_logger().info("SignDataSubscriber node has been started.")

        self.sign = ""

    def sign_data_callback(self, msg):
        self.get_logger().info("Received bounding_boxes message.")
        if not self.data_saved:
            for box in msg.bounding_boxes:
                if box.probability >= 0.95 and box.class_id in ["A1", "A2", "A3"]:
                    # Ensure the directory exists
                    if not os.path.exists(self.directory_path):
                        # self.get_logger().warn(
                        #     f"Directory does not exist, creating directory: {self.directory_path}"
                        # )
                        try:
                            os.makedirs(self.directory_path)
                        except OSError as e:
                            self.get_logger().error(
                                f"Failed to create directory: {self.directory_path}, error: {e}"
                            )
                            return

                    # Write to the file
                    try:
                        with open(self.file_path, "w") as file:
                            file.write(box.class_id + "\n")
                        self.data_saved = True
                        self.get_logger().info(
                            f"Saved sign data: {box.class_id} with probability: {box.probability}"
                        )

                        # Allow time for the file operation to complete
                        time.sleep(1)  # 추가

                        # Shutdown the node
                        break  # Save only the first matching data
                    except IOError as e:
                        self.get_logger().error(
                            f"Failed to open file: {self.file_path}, error: {e}"
                        )
                        return

    def callback_erp(self, msg):
        self.go.v = msg.speed

    def pickup_stanley(self):
        msg = ControlMessage()
        if len(self.x_list) != 0:
            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                self.go, self.x_list, self.y_list, self.yaw_list
            )
            msg.steer = int(steer)
            msg.speed = 4
            msg.gear = 0

            # self.target_idx가 20에 도달하면 pickup_finished를 True로 설정
            if self.target_idx >= 20 and not self.pickup_finished:
                self.pickup_finished = True
                print("Pickup finished!", flush=True)

            mission_finish = self.pickup_finished
            print(steer, self.target_idx, mission_finish)

            return msg, mission_finish

        else:
            print("not path")

    def odom_callback(self, msg):

        self.pickup_stanley()

        self.go.x = msg.pose.pose.position.x
        self.go.y = msg.pose.pose.position.y
        _, _, self.go.yaw = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )

        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y

        current_mission = "driving"  # 기본 미션

        # 자차의 위치에 해당하는 Lanelet 찾기
        for lanelet in self.lanelet_map.lanelets:
            if self.is_point_in_lanelet(car_x, car_y, lanelet):
                if lanelet.mission:
                    current_mission = lanelet.mission

                    # mission이 pickup인 경우, centerline의 첫 번째 노드 좌표를 퍼블리시
                    if current_mission == "pickup" and lanelet.centerline:
                        first_node_id = lanelet.centerline.node_refs[0]
                        node_x, node_y = self.lanelet_map.get_node_position(
                            first_node_id
                        )
                        if node_x is not None and node_y is not None:
                            # pickup_start 퍼블리시
                            self.pickup_start = PointStamped()
                            self.pickup_start.header.stamp = (
                                self.get_clock().now().to_msg()
                            )
                            self.pickup_start.header.frame_id = "map"
                            self.pickup_start.point.x = node_x
                            self.pickup_start.point.y = node_y
                            self.pickup_start.point.z = 0.0
                            self.pickup_start_pub.publish(self.pickup_start)

                            # 고정된 pickup_goal 퍼블리시
                            self.pickup_goal.header.stamp = (
                                self.get_clock().now().to_msg()
                            )
                            self.pickup_goal_pub.publish(self.pickup_goal)
                break

        # mission 토픽 퍼블리시
        mission_msg = String()
        mission_msg.data = current_mission
        self.mission_pub.publish(mission_msg)

    def publish_path(self):
        if self.pickup_start is None or self.path_published_once:
            return

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"

        # start와 goal 사이를 여러 개의 점으로 연결
        start_pose = PoseStamped()
        start_pose.header = self.pickup_start.header
        start_pose.pose.position = self.pickup_start.point
        start_pose.pose.orientation.w = 1.0  # 기본적으로 방향을 지정

        goal_pose = PoseStamped()
        goal_pose.header = self.pickup_goal.header
        goal_pose.pose.position = self.pickup_goal.point
        goal_pose.pose.orientation.w = 1.0

        # 두 점을 더 촘촘하게 연결하기 위해 선형 보간
        num_intermediate_points = 20  # 경로를 이루는 중간 점의 개수 (간격 조정 가능)
        for i in range(num_intermediate_points + 1):
            t = i / num_intermediate_points
            intermediate_pose = PoseStamped()
            intermediate_pose.header = path.header
            intermediate_pose.pose.position.x = (
                start_pose.pose.position.x * (1 - t) + goal_pose.pose.position.x * t
            )
            intermediate_pose.pose.position.y = (
                start_pose.pose.position.y * (1 - t) + goal_pose.pose.position.y * t
            )
            intermediate_pose.pose.position.z = 0.0  # 고정된 z 값

            # orientation 계산 (start에서 goal로의 방향)
            delta_x = goal_pose.pose.position.x - start_pose.pose.position.x
            delta_y = goal_pose.pose.position.y - start_pose.pose.position.y
            yaw = math.atan2(delta_y, delta_x)
            intermediate_pose.pose.orientation.z = math.sin(yaw * 0.5)
            intermediate_pose.pose.orientation.w = math.cos(yaw * 0.5)

            # x, y, yaw 값을 리스트에 저장
            self.x_list.append(intermediate_pose.pose.position.x)
            self.y_list.append(intermediate_pose.pose.position.y)
            self.yaw_list.append(yaw)

            path.poses.append(intermediate_pose)

        self.pickup_path_pub.publish(path)

        # x, y, yaw 리스트를 한 번만 출력
        # if not self.path_published_once:
        #     # self.get_logger().info(f"x_list: {self.x_list}")
        #     # self.get_logger().info(f"y_list: {self.y_list}")
        #     self.path_published_once = True  # 출력 후 플래그 설정

    def is_point_in_lanelet(self, x, y, lanelet):
        if lanelet.centerline is None:
            return False

        # lanelet의 centerline이 구성하는 폴리곤을 사용하여 자차가 해당 lanelet에 있는지 확인
        polygon = []
        if lanelet.left_way:
            polygon += [
                self.lanelet_map.get_node_position(nid)
                for nid in lanelet.left_way.node_refs
            ]
        if lanelet.right_way:
            polygon += [
                self.lanelet_map.get_node_position(nid)
                for nid in lanelet.right_way.node_refs
            ]

        if len(polygon) < 3:  # 삼각형 이상이 되어야 폴리곤이 성립됨
            return False

        # Point in Polygon (PIP) 알고리즘 적용
        inside = self.is_point_in_polygon(x, y, polygon)
        return inside

    def is_point_in_polygon(self, x, y, polygon):
        inside = False
        n = len(polygon)
        p1x, p1y = polygon[0]
        for i in range(n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside


def main(args=None):
    rclpy.init(args=args)
    node = PickupDeliveryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
