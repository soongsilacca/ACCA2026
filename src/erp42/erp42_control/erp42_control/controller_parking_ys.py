# ros2
import rclpy
from rclpy.qos import qos_profile_system_default

# msg
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from erp42_msgs.msg import ControlMessage

# tf
from tf_transformations import *

# stanley
from stanley import Stanley

# DB
try:
    from DB import DB
except Exception as e:
    print(e)

# utils
import time
from enum import Enum
import numpy as np
import math as m
## TODO
# cropbox 조정 / 첫 포즈 거리 조정

class SpeedSupporter:
    def __init__(self, node):
        # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value

        self.he_gain = node.declare_parameter(
            "/speed_supporter/he_gain_parking", 50.0
        ).value
        self.ce_gain = node.declare_parameter(
            "/speed_supporter/ce_gain_parking", 30.0
        ).value

        self.he_thr = node.declare_parameter(
            "/speed_supporter/he_thr_parking", 0.001
        ).value
        self.ce_thr = node.declare_parameter(
            "/speed_supporter/ce_thr_parking", 0.002
        ).value

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
        self.p_gain = node.declare_parameter(
            "/stanley_controller/p_gain_parking", 2.07
        ).value
        self.i_gain = node.declare_parameter(
            "/stanley_controller/i_gain_parking", 0.85
        ).value

        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0

        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )

    def PIDControl(self, speed, desired_value, min, max):

        self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (
            self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        dt = self.current - self.last
        self.last = self.current

        err = desired_value - speed
        # self.d_err = (err - self.p_err) / dt
        self.p_err = err
        self.i_err += self.p_err * dt * (0.0 if speed == 0 else 1.0)

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(self.speed, min, max))


class Parking_state(Enum):
    SEARCH = 0
    PARKING = 1
    STOP = 2
    RETURN = 3
    FINISH = 4


class Parking:
    def __init__(self, node):
        self.node = node

        # search_path
        self.search_path_db = DB("P_search_path.db")  # kcity
        self.search_path = self.search_path_db.read_db_n("Path", "x", "y", "yaw")

        # print(self.search_path)
        #### parking_path ####
        dx, dy = 1.172, 2.6

        self.parking_path_db_1 = DB("P_parking_path_1.db")
        self.parking_path_db_2 = DB("P_parking_path_2.db")
        self.parking_path_db_3 = DB("P_parking_path_3.db")
        self.parking_path_db_4 = DB("P_parking_path_4.db")
        self.parking_path_db_5 = DB("P_parking_path_5.db")
        self.parking_path_db_6 = DB("P_parking_path_6.db")

        self.parking_path_1 = self.parking_path_db_1.read_db_n("Path", "x", "y", "yaw")
        self.parking_path_2 = self.parking_path_db_2.read_db_n("Path", "x", "y", "yaw")
        self.parking_path_3 = self.parking_path_db_3.read_db_n("Path", "x", "y", "yaw")
        self.parking_path_4 = self.parking_path_db_4.read_db_n("Path", "x", "y", "yaw")
        self.parking_path_5 = self.parking_path_db_5.read_db_n("Path", "x", "y", "yaw")
        self.parking_path_6 = self.parking_path_db_6.read_db_n("Path", "x", "y", "yaw")

        def shift_path(path, n):
            return [(x + n * dx, y + n * dy, yaw) for (x, y, yaw) in path]

        # self.parking_path_2 = shift_path(self.parking_path_1, 1)
        # self.parking_path_3 = shift_path(self.parking_path_1, 2)
        # self.parking_path_4 = shift_path(self.parking_path_1, 3)
        # self.parking_path_5 = shift_path(self.parking_path_1, 4)
        # self.parking_path_6 = shift_path(self.parking_path_1, 5)

        self.return_path_1 = self.parking_path_1[::-1]
        self.return_path_2 = self.parking_path_2[::-1]
        self.return_path_3 = self.parking_path_3[::-1]
        self.return_path_4 = self.parking_path_4[::-1]
        self.return_path_5 = self.parking_path_5[::-1]
        self.return_path_6 = self.parking_path_6[::-1]

        # instance
        self.state = Parking_state.SEARCH
        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)

        self.odometry = None

        self.timer = None
        self.cones = []
        self.goal = 0  # 목표 주차 경로 (1, 2, 3,4,5,6 중 하나)
        self.blocked_paths = set()

        # subscriber
        self.node.create_subscription(
            PoseArray, "/cone_pose_map", self.cone_callback, 10
        )

        # publisher (visualization)
        self.pub_path = self.node.create_publisher(
            Path, "mission/parking", qos_profile_system_default
        )

    def cone_callback(self, msg):
        self.cones = self.cones[-10:]  # Clear the list before appending new points
        for pose in msg.poses:
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = pose.position.z
            self.cones.append(point)

    def control_parking(self, odometry):
        self.odometry = odometry
        
        msg = ControlMessage()

        if self.state == Parking_state.SEARCH:

            path_x = [p[0] for p in self.search_path]
            path_y = [p[1] for p in self.search_path]
            path_yaw = [p[2] for p in self.search_path]
            self.publish_path_msg(path_x, path_y, path_yaw)

            steer, target_idx, hdr, ctr = self.st.stanley_control(
                odometry,
                path_x,
                path_y,
                path_yaw,
                h_gain=0.5,
                c_gain=0.24,
            )
            adapted_speed = self.ss.adaptSpeed(
                10, hdr, ctr, min_value=7, max_value=12
            )  # 에러(hdr, ctr) 기반 목표 속력 조정
            speed = self.pid.PIDControl(
                self.odometry.v * 3.6, adapted_speed, min=7, max=12
            )  # speed 조정 (PI control)
            brake = self.cacluate_brake(adapted_speed)  # brake 조정

            msg.speed = int(speed) * 10
            msg.steer = int(m.degrees((-1) * steer) * 1e3)
            msg.brake = int(brake)
            msg.gear = 2

            if self.timer is None:
                self.timer = self.node.create_timer(0.1, self.find_parking_path)
            if target_idx >= len(path_x) - 10:
                self.timer.cancel()
                return msg, True

        elif self.state == Parking_state.PARKING:
            print("[PARKING] 주차 경로 따라 주차 중...")
            # 딱 한 번만 1초간 정지 동작
            if not hasattr(self, "parking_wait_flag"):
                self.parking_wait_flag = False
                self.parking_wait_start = 0.0

            if not self.parking_wait_flag:
                self.parking_wait_flag = True
                self.parking_wait_start = time.time()
                msg = ControlMessage(
                    mora=0, estop=1, gear=0, speed=0, steer=0, brake=200
                )
                return msg, False
            elif time.time() - self.parking_wait_start < 1.0:
                msg = ControlMessage(
                    mora=0, estop=1, gear=0, speed=0, steer=0, brake=200
                )
                return msg, False
            # 1초가 지난 후에는 주자 동작
            path = [
                self.parking_path_1,
                self.parking_path_2,
                self.parking_path_3,
                self.parking_path_4,
                self.parking_path_5,
                self.parking_path_6,
            ][self.goal - 1]
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            path_yaw = [p[2] for p in path]
            self.publish_path_msg(path_x, path_y, path_yaw)

            steer, target_idx, hdr, ctr = self.st.stanley_control(
                odometry,
                path_x,
                path_y,
                path_yaw,
                h_gain=1.5,
                c_gain=1.2,
                reverse=False,  # 전진 주차
            )
            adapted_speed = self.ss.adaptSpeed(
                10, hdr, ctr, min_value=7, max_value=12
            )  # 에러(hdr, ctr) 기반 목표 속력 조정
            speed = self.pid.PIDControl(
                self.odometry.v * 3.6, adapted_speed, min=7, max=12
            )  # speed 조정 (PI control)
            brake = self.cacluate_brake(adapted_speed)  # brake 조정

            msg.speed = int(speed) * 10
            msg.steer = int(m.degrees((-1) * steer)*1e3)
            msg.gear = 2  # 전진 주차
            msg.brake = int(brake)

            if target_idx >= len(path_x) - 3:
                self.state = Parking_state.STOP

        elif self.state == Parking_state.STOP:
            print("[STOP] 주차 완료, 3초간 정지 중...")
            # 딱 한 번만 3초간 정지 동작
            if not hasattr(self, "parked_wait_flag"):
                self.parked_wait_flag = False
                self.parked_wait_start = 0.0

            if not self.parked_wait_flag:
                self.parked_wait_flag = True
                self.parked_wait_start = time.time()
                msg = ControlMessage(
                    mora=0, estop=1, gear=0, speed=0, steer=0, brake=200
                )
                return msg, False
            elif time.time() - self.parked_wait_start < 3.0:
                msg = ControlMessage(
                    mora=0, estop=1, gear=0, speed=0, steer=0, brake=200
                )
                return msg, False
            # 3초가 지난 후에는 후진 동작
            self.state = Parking_state.RETURN
        elif self.state == Parking_state.RETURN:
            print("[RETURN] 리턴 경로 따라 후진 중...")
            path = [
                self.return_path_1,
                self.return_path_2,
                self.return_path_3,
                self.return_path_4,
                self.return_path_5,
                self.return_path_6,
            ][self.goal - 1]
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            path_yaw = [p[2] for p in path]
            self.publish_path_msg(path_x, path_y, path_yaw)

            steer, target_idx, hdr, ctr = self.st.stanley_control(
                odometry,
                path_x,
                path_y,
                path_yaw,
                h_gain=2.0,
                c_gain=1.5,
                reverse=True,  # 후진
            )
            adapted_speed = self.ss.adaptSpeed(
                12, hdr, ctr, min_value=8, max_value=14
            )  # 에러(hdr, ctr) 기반 목표 속력 조정
            speed = self.pid.PIDControl(
                self.odometry.v * 3.6, adapted_speed, min=8, max=14
            )  # speed 조정 (PI control)
            brake = self.cacluate_brake(adapted_speed)  # brake 조정

            msg.speed = int(speed) * 10
            msg.steer = int(m.degrees((-1) * steer)*1e3)
            msg.gear = 0  # 후진
            msg.brake = int(brake)

            if target_idx >= len(path_x) - 3:
                return msg, True
            
        print(target_idx)
        return msg, False

    def find_parking_path(self):
        if self.state == Parking_state.SEARCH:
            paths = [
                self.parking_path_1,
                self.parking_path_2,
                self.parking_path_3,
                self.parking_path_4,
                self.parking_path_5,
                self.parking_path_6,
            ]

            min_dist_threshold = 0.3  # 장애물과의 최소 거리
            over_threshold = []

            for idx, path in enumerate(paths, 1):
                if idx in self.blocked_paths:  # 이미 차단된 경로는 건너뜀
                    continue

                is_blocked = False
                for cone in self.cones:
                    for px, py, _ in path:
                        dist = ((cone.x - px) ** 2 + (cone.y - py) ** 2) ** 0.5
                        if dist < min_dist_threshold:
                            is_blocked = True
                            break
                    if is_blocked:
                        break

                if is_blocked:
                    self.blocked_paths.add(idx)  # 차단된 경로로 등록
                else:
                    over_threshold.append(idx)

            if len(over_threshold) > 1:
                print(
                    f"[경고] 주차 경로가 {len(over_threshold)}개 이상 선택되었습니다. {over_threshold} 중 하나를 선택합니다."
                )
                cur_x = self.odometry.x
                cur_y = self.odometry.y

                selected = None
                for idx in over_threshold:
                    path = paths[idx - 1]
                    if path:
                        px, py, _ = path[0]
                        dist = ((cur_x - px) ** 2 + (cur_y - py) ** 2) ** 0.5
                        if dist <= 1.0: # 1.0m 이내에 있는 경로 우선 선택
                            selected = idx
                            break

                if selected is not None:
                    self.goal = selected
                    self.timer.cancel()
                    self.state = Parking_state.PARKING
                    print(f"주차 경로 {self.goal}을 선택했습니다. (0.3m 이내)")
                else:
                    print(
                        "[경고] 0.3m 이내에 가까운 주차 경로가 없습니다. 경로를 선택하지 않습니다."
                    )

            elif len(over_threshold) == 0:
                print("[경고] 유효한 주차 경로가 없습니다. 모두 차단됨.")
            else:
                self.goal = over_threshold[0]
                self.timer.cancel()
                self.state = Parking_state.PARKING
                print(f"주차 경로 {self.goal}을 선택했습니다.")

    def cacluate_brake(
        self, adapted_speed
    ):  # brake 값 정하는 알고리즘 좀 더 정교하게 생각
        if self.odometry.v * 3.6 >= adapted_speed:
            brake = (abs(self.odometry.v * 3.6 - adapted_speed) / 20.0) * 200
            brake = np.clip(brake, 0, 100)
        else:
            brake = 0
        return brake

    def publish_path_msg(self, path_x, path_y, path_yaw):
        path_msg = Path()
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for x, y, yaw in zip(path_x, path_y, path_yaw):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            yaw_f = float(yaw)
            qx, qy, qz, qw = 0.0, 0.0, np.sin(yaw_f / 2), np.cos(yaw_f / 2)
            pose.pose.orientation.x = float(qx)
            pose.pose.orientation.y = float(qy)
            pose.pose.orientation.z = float(qz)
            pose.pose.orientation.w = float(qw)
            path_msg.poses.append(pose)
        self.pub_path.publish(path_msg)
