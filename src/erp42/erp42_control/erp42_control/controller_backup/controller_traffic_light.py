import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from erp42_msgs.msg import ControlMessage, SerialFeedBack  # SerialFeedBack 추가
from darknet_ros_msgs.msg import BoundingBoxes
from tf_transformations import euler_from_quaternion
import math
import numpy as np
from stanley import Stanley
from visualization_msgs.msg import Marker
import time


class SpeedSupporter:
    def __init__(self, node):
        # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value

        self.he_gain = node.declare_parameter("/speed_supporter/he_gain_traffic", 40.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain_traffic", 20.0).value

        self.he_thr = node.declare_parameter("/speed_supporter/he_thr_traffic",0.05).value
        self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr_traffic",0.001).value

    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self, value, hdr, ctr, min_value, max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)
        return res

class PID():
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain_traffic", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain_traffic", 0.85).value
       
        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0
        
        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
    def PIDControl(self, speed, desired_value, min, max):

        self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        dt = self.current - self.last
        self.last = self.current

        err = desired_value - speed
        # self.d_err = (err - self.p_err) / dt 
        self.p_err = err
        self.i_err += self.p_err * dt  * (0.0 if speed == 0 else 1.0)

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(self.speed, min, max))


        
class Trafficlight:
    def __init__(self, node):
        self.node = node

        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)

        self.target_idx = 0
        self.pre_stanley_idx = 0
        
        # 신호등 관련 상태 변수 초기화
        self.current_signal = None
        self.signal_count = 0
        self.red_light_detected = False
        self.green_light_detected = False
        self.mission_finish = False
        self.current_odometry = None
        self.start_code = True  # 일단 버리자

        self.red_light_time = None
        self.green_light_time = None
        self.none_light_time = None
        self.recent_signals = []
        
        self.target_speed = 8
        
        # 신호등 인덱스 0~6 !!!!!!!!!!!!!!!!!!
        self.current_index = 3

        self.last_signal_time = time.time()

        # 퍼블리셔 설정
        self.control_pub = self.node.create_publisher(ControlMessage, "cmd_msg", 10)
        
        # 신호등 인식 데이터 구독
        self.traffic_light_sub = self.node.create_subscription(
            BoundingBoxes, "bounding_boxes", self.traffic_light_callback, 10
        )

        self.traffic_light_sections = [
            # 각 구간에 대한 빨간불 ID, 초록불 ID 설정, 왼빨강불 오초록불

            #돌계 테스트용
            # ({"A3", "A2"}, {"A1"}),  #첫  번째 신호등 (current_index=1)
            
            (
                {"1401","1301"},           # 첫 번째 신호등 (current_index=0) 
                {"1405","1300", "1400","1402"},
            ),                             
            ({"1401","1301"}, { "1402", "1405", "1400", "1300"}),  # 두 번째 신호등 (current_index=1)
            ({"1401","1301"}, {"1400", "1405", "1300","1402","1302"}),  # 세 번째 신호등 (current_index=2)
            (
                {"1400", "1401", "1300","1301"},
                {"1402","1403", "1303", "1404","1302"},
            ),                              # 네 번째 신호등 (current_index=3)
            ({"1301", "1401"}, {"1303", "1403", "1404"}),           # 다섯 번째 신호등 (current_index=4)
            
            ({"1401", "1301"}, {"1400","1405", "1300", "1402", "1302", "1404"}),  # 여섯 번째 신호등 (current_index=5)
            ({"1401", "1301"}, {"1400","1405", "1300", "1402", "1302", "1404"}),  # 일곱 번째 신호등 (current_index=6)
        ] 



        self.count = 0




        
    # 신호등의 초록불, 빨간불 분류
    def get_current_traffic_light_ids(self):
        red_ids = self.traffic_light_sections[self.current_index][0]
        green_ids = self.traffic_light_sections[self.current_index][1]
        return red_ids, green_ids

        
    # 신호 확정
    def traffic_light_callback(self, msg):
        self.last_signal_time = time.time()
        if self.start_code:
            detected_signal = None
            confidence = 0.0

            # 현재 구간의 신호등 ID 가져오기
            red_ids, green_ids = self.get_current_traffic_light_ids()

            for box in msg.bounding_boxes:
                
                detected_signal = box.class_id  


                confidence = box.probability  # 신뢰도 사용 X

                if detected_signal in red_ids:

                    self.red_light_detected = True
                    self.red_light_time = time.time()
                    self.recent_signals.append("red")

                    self.none_light_time = None

                elif detected_signal in green_ids:

                    self.green_light_detected = True
                    self.green_light_time = time.time()
                    self.recent_signals.append("green")

                    self.none_light_time = None
                else:  # 해당되지 않는 신호 잡힐 때
                    self.none_light_time = time.time()
                    self.recent_signals.append("none")

                self.get_real_light()  # 잡히는 것 중에 알맞는 방식 채택

    # 초록불이나 빨간불중 하나만 잡히면 확정시킴
    def get_real_light(self):
        
        if self.red_light_detected and not self.green_light_detected:
            self.current_signal = "red"
            return
        if self.green_light_detected and not self.red_light_detected:
            self.current_signal = "green"
            return

        # 카운트를 세서 기준치 이상이면 확정
        count_threshold = 6
        
        # 최근 10번의 카운트중 최근 같은 카운트가 6번 이상이면 신호 전환
        if len(self.recent_signals) > 10:
            self.recent_signals = self.recent_signals[-10:]

        if (
            self.recent_signals.count("red") >= count_threshold
            and self.current_signal == "green"
        ):
            self.current_signal = "red"
            self.time_reset()
            return
        if (
            self.recent_signals.count("green") >= count_threshold
            and self.current_signal == "red"
        ):
            self.current_signal = "green"
            self.time_reset()
            return

        # 계속해서 하나만 잡히면 신호 확정
        if self.recent_signals.count("red") == 10:
            self.current_signal = "red"
            self.time_reset()
            return
        if self.recent_signals.count("green") == 10:
            self.current_signal = "green"
            self.time_reset()
            return


        # 확정되고 타이머로 하나가 잡히면 확정
        time_threshold = 2
        if (
            self.current_signal is not None
            and self.green_light_time is not None
            and self.red_light_time is not None
        ):
            if (
                time.time() - self.red_light_time > time.time() - self.green_light_time
            ):  # 최근에 초록불이 켜짐
                if (
                    time.time() - self.green_light_time > time_threshold
                ):  # 초록불이 켜지고 2초동안 다른게 안잡히면
                    self.current_signal = "green"
                    self.time_reset()
                    return
            else:  # 최근에 빨간불이 켜짐
                if (
                    time.time() - self.red_light_time > time_threshold
                ):  # 빨간불이 잡히고 2초동안 다른게 안잡힘
                    self.current_signal = "red"
                    self.time_reset()
                    return

        if (
            self.current_signal is not None and self.none_light_time is not None
        ):  # 한번 확정되고 나서 오랫동안 아무것도 안잡히면
            if time.time() - self.none_light_time > 3:
                self.current_signal = None
                self.green_light_detected = False
                self.red_light_detected = False
                self.time_reset()

    def time_reset(self):
        self.red_light_time = None
        self.green_light_time = None
        
    

        
    def control_traffic_light(self, odometry, path):
        if self.start_code == False:
            self.start_code = True


        msg = ControlMessage()
        steer, self.target_idx, hdr, ctr = self.st.stanley_control(
            odometry,
            path.cx,
            path.cy,
            path.cyaw,
            h_gain=0.5,
            c_gain=0.24,
        )

        current_distance_to_stop_line = len(path.cx) - self.target_idx

        # 빨간불 감지 시 처리
        if self.current_signal == "red":
            self.count = 0 # red 신로 잡히면 count 0으로 초기화

            if current_distance_to_stop_line <= 10:
                speed = 0
                estop = 1
                self.node.get_logger().info(f"빨간불 확인: 차량 정지, 현재속도 = {msg.speed}")

                if time.time() - self.last_signal_time > 3:
                    self.node.get_logger().info("신호없음, 그냥 출발")
                    self.red_light_detected = False
                    self.green_light_detected = False
                    self.current_signal = None
                    
            else:
                self.target_speed = (current_distance_to_stop_line / len(path.cx)) * 15
                self.target_speed = int(np.clip(self.target_speed, 6, 8))
                adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, min_value=6, max_value=8)
                speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed, min=6, max=8)
                estop = 0
                self.node.get_logger().info(f"빨간불 감지: 차량 감속, 현재속도 = {msg.speed}")

        # 초록불 감지된 경우 처리
        elif self.current_signal == "green":
            self.count = 0 # green 신로 잡히면 count 0으로 초기화

            if self.current_index == 6:
                self.target_speed = 16
                adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, min_value=14, max_value=16)
                speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed, min=14, max=16)
                estop = 0
                self.node.get_logger().info(f"초록불 감지: 주행 지속, 속도 유지 = {msg.speed}")

            else:
                self.target_speed = 10
                adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, min_value=8, max_value=10)
                speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed, min=8, max=10)
                estop = 0
                self.node.get_logger().info(f"초록불 감지: 주행 지속, 속도 유지 = {msg.speed}")

            if current_distance_to_stop_line <= 5:
                self.mission_finish = True
                self.node.get_logger().info("초록불 주행 중 미션 완료")

        # 신호 확정 불가 시 처리
        else:

            self.count += 1 #아무 신호도 안 잡히면 count를 0.1초마다 1씩 증가
            if self.current_index == 6:
                if current_distance_to_stop_line <= 5:
                    if self.count >= 40:
                        self.target_speed = 16
                        adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, min_value=14, max_value=16)
                        speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed, min=14, max=16)
                        estop = 0
                        self.mission_finish = True
                        self.node.get_logger().info("불가피한 미션 완료")

                    else:
                        speed = 0
                        estop = 1

                else:
                    self.target_speed = 16
                    adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, min_value=14, max_value=16)
                    speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed, min=14, max=16)
                    estop = 0
                    self.node.get_logger().info("신호등 확정불가")


            elif self.current_index == 4:
                if current_distance_to_stop_line <= 5:
                    if self.count >= 700:
                        self.target_speed = 10
                        adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, min_value=8, max_value=10)
                        speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed, min=8, max=10)
                        estop = 0
                        self.mission_finish = True
                        self.node.get_logger().info("불가피한 미션 완료")

                    else:
                        speed = 0
                        estop = 1

                else:
                    self.target_speed = 10
                    adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, min_value=8, max_value=10)
                    speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed, min=8, max=10)
                    estop = 0
                    self.node.get_logger().info("신호등 확정불가")


            else:
                if current_distance_to_stop_line <= 5:
                    if self.count >= 40:
                        self.target_speed = 10
                        adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, min_value=8, max_value=10)
                        speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed, min=8, max=10)
                        estop = 0
                        self.mission_finish = True
                        self.node.get_logger().info("불가피한 미션 완료")

                    else:
                        speed = 0
                        estop = 1

                else:
                    self.target_speed = 10
                    adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, min_value=8, max_value=10)
                    speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed, min=8, max=10)
                    estop = 0
                    self.node.get_logger().info("신호등 확정불가")


            print(self.count)

        msg.steer = int(math.degrees(-1 * steer))
        msg.speed = int(speed) * 10
        msg.gear = 2
        msg.estop = estop

        if self.mission_finish:
            if self.current_index != 6:
                self.current_index += 1
                self.reset_traffic_light()
                self.node.get_logger().info(
                    f"미션 완료: 다음 신호등 구간으로 이동 (Index: {self.current_index})"
                )
                
            self.start_code = False
            return msg, True  # 미션 완료

        return msg, False  # 미션이 완료되지 않았을 때 반환
    
    def reset_traffic_light(self):
        # 신호 관련 상태를 초기화하는 함수
        self.red_light_detected = False
        self.green_light_detected = False
        self.signal_count = 0  # 신호 카운트 리셋
        self.current_signal = None  # 현재 신호 리셋
        self.mission_finish = False
        self.recent_signals = []