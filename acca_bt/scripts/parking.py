#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseArray, Point, Quaternion
from nav_msgs.msg import Path, Odometry
import math
import time
import tf_transformations # for quaternion conversions if needed, or manual math
import time

class ParkingExecutor(Node):
    def __init__(self):
        super().__init__('parking_executor')
        
        # BT 명령 및 상태 통신
        self.cmd_sub = self.create_subscription(String, '/parking_command', self.command_callback, 10)
        self.status_pub = self.create_publisher(String, '/parking_status', 10)
        
        # 주차 구역 및 장애물 정보 구독
        self.create_subscription(PoseStamped, '/parking/A', lambda msg: self.set_spot('A', msg), 10)
        self.create_subscription(PoseStamped, '/parking/B', lambda msg: self.set_spot('B', msg), 10)
        self.create_subscription(PoseStamped, '/parking/C', lambda msg: self.set_spot('C', msg), 10)
        self.create_subscription(PoseArray, '/cone_pose_map', self.obstacle_callback, 10)
        
        # [NEW] 내 차 위치 구독 및 경로 시각화 토픽
        self.create_subscription(Odometry, '/odometry/global', self.odom_callback, 10)
        self.vis_pub = self.create_publisher(Path, '/parking/trajectory_vis', 10)

        self.current_pose = None
        self.spots = {'A': None, 'B': None, 'C': None}
        self.obstacles = []
        self.is_parking = False

        self.get_logger().info("Smart Parking Executor Ready.")

    def set_spot(self, name, msg):
        self.spots[name] = msg

    def obstacle_callback(self, msg):
        self.obstacles = msg.poses

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def check_availability(self, spot_name):
        spot = self.spots[spot_name]
        if not spot: return False # 정보 없으면 패스
        
        for obs in self.obstacles:
            dist = math.sqrt((obs.position.x - spot.pose.position.x)**2 + 
                             (obs.position.y - spot.pose.position.y)**2)
            if dist < 2.5: # 2.5m 이내에 장애물 있으면 점유된 것으로 판단
                return False
        return True

    def command_callback(self, msg):
        if self.is_parking: return
        
        self.get_logger().info("Searching for an available parking spot...")
        target = None
        for name in ['A', 'B', 'C']:
            if self.check_availability(name):
                target = name
                break
        
        if target:
            self.get_logger().info(f"Target Found: Spot {target}. Starting Maneuver!")
            self.execute_parking_sequence(target)
        else:
            self.get_logger().warn("No available parking spots found!")
            self.status_pub.publish(String(data="FAILURE"))

    def send_control(self, speed, steer, gear):
        # 실제 차량 제어 로직 (ERP42 전송부)
        self.get_logger().info(f"CTRL -> Spd: {speed}, Steer: {steer}, Gear: {gear}")

    def execute_parking_sequence(self, spot_id):
        self.is_parking = True
        self.status_pub.publish(String(data="RUNNING"))

        # [NEW] 주차 경로 미리 계산해서 Rviz에 보여주기
        if self.current_pose:
            self.visualize_path()

        # [기동 시나리오 - 5단계]
        steps = [
            (1.0, 0, 2, 3.0),   # 1. 전진 (3초)
            (0.0, 0, 2, 1.0),   # 2. 정지 (1초)
            (1.0, -28, 0, 3.0), # 3. 우측 후진 (3초)
            (1.0, 28, 0, 3.0),  # 4. 좌측 후진 (3초)
            (0.0, 0, 2, 2.0)    # 5. 최종 정지 (2초)
        ]

        for spd, str_ang, gr, duration in steps:
            self.send_control(spd, str_ang, gr)
            time.sleep(duration)

        self.get_logger().info(f"Successfully parked in Spot {spot_id}!")
        self.status_pub.publish(String(data="SUCCESS"))
        self.is_parking = False

    def visualize_path(self):
        """ 현재 위치 기준으로 주차 시나리오를 시뮬레이션해서 경로를 그린다. """
        if not self.current_pose: return

        # 초기 상태 (x, y, yaw)
        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y
        
        # 쿼터니언 -> Yaw 변환
        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        curr_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 시나리오 단계 정의 (execute_parking_sequence와 동일해야 함)
        # (speed_kph, steer_deg, gear, duration)
        steps = [
            (3.0, 0, 2, 3.0),   # 1. 전진 (3초) -> gear 2(D)
            (0.0, 0, 2, 1.0),   # 2. 정지
            (3.0, -28, 0, 3.0), # 3. 우측 후진 (3초) -> gear 0(R)
            (3.0, 28, 0, 3.0),  # 4. 좌측 후진 (3초) -> gear 0(R)
            (0.0, 0, 2, 2.0)    # 5. 정지
        ]

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        dt = 0.1 # 0.1초 단위로 점 찍기
        WB = 1.04 # 휠베이스 (ERP42 대략값)

        for speed_kph, steer_deg, gear, duration in steps:
            # 속도 및 방향 설정
            speed_ms = (speed_kph / 3.6)
            if gear == 0: speed_ms = -speed_ms # 후진이면 속도 음수
            
            steer_rad = math.radians(steer_deg)
            
            # duration 동안 dt 간격으로 적분
            n_steps = int(duration / dt)
            if n_steps == 0: n_steps = 1
            
            for _ in range(n_steps):
                # Bicycle Path Model
                curr_x += speed_ms * math.cos(curr_yaw) * dt
                curr_y += speed_ms * math.sin(curr_yaw) * dt
                curr_yaw += (speed_ms / WB) * math.tan(steer_rad) * dt
                
                # Path에 점 추가
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = curr_x
                pose.pose.position.y = curr_y
                
                # Yaw -> Quaternion 변환 (시각화용)
                cy = math.cos(curr_yaw * 0.5)
                sy = math.sin(curr_yaw * 0.5)
                pose.pose.orientation.w = cy
                pose.pose.orientation.z = sy
                
                path_msg.poses.append(pose)

        self.vis_pub.publish(path_msg)
        self.get_logger().info(f"Published visual parking path with {len(path_msg.poses)} points.")

def main(args=None):
    rclpy.init(args=args)
    node = ParkingExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()