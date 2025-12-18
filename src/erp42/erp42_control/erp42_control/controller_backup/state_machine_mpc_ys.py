#!/usr/bin/rrnv python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from erp42_msgs.msg import SerialFeedBack, ControlMessage
from std_msgs.msg import Float64, Int64, Float32, String

from stanley import Stanley
# from .mpc_node import MPC
from .mpc_node_tunning_2 import MPC
from DB import DB
import numpy as np
import math as m
from Modifier_param import ParamConfigurer
# from LifecycleController import LifecycleController
from enum import Enum
import threading

from controller_obstacle_ys import Obstacle
from controller_uturn import Uturn
from controller_parking_ys import Parking


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return qx, qy, qz, qw


class PID:
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain", 0.85).value
        # self.p_gain = node.declare_parameter("/stanley_controller/p_gain", 1.0).value
        # self.i_gain = node.declare_parameter("/stanley_controller/i_gain", 0.05).value

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

        # self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (
        #     self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        # )
        # dt = self.current - self.last
        # self.last = self.current

        err = desired_value - speed
        # self.d_err = (err - self.p_err) / dt

        self.p_err = err
        # self.i_err += self.p_err * dt * (0.0 if speed == 0 else 1.0)
        # if self.i_err > 5.0:
        #     self.i_err = 5.0
        # if self.i_err < -5.0:
        #     self.i_err = -5.0

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        # self.speed = speed + (self.p_gain * self.p_err) 

        return int(np.clip(self.speed, min, max))


class SpeedSupporter:
    def __init__(self, node):
        # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value

        self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 50.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 30.0).value

        self.he_thr = node.declare_parameter("/speed_supporter/he_thr", 0.001).value
        self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr", 0.002).value

    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self, value, hdr, ctr, min_value, max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)
        return res


class State(Enum):
    ########### YS MPC ############
    # A1A2 = "driving_a"
    # A2A3 = "driving_stanley_b"
    # A3A4 = "driving_c"
    # B1B2 = "uturn_d"
    # A4A5 = "driving_e"
    # A5A6 = "obstacle_f"
    ########### YS MPC ############
    # ssupark
    # A1A2 = "driving_a"
    # A2A3 = "parking_b"
    # A3A4 = "driving_c"
    # B1B2 = "uturn_d"
    # A4A5 = "driving_e"

    # A1A2="driving_a"
    # A2A3="driving_b"
    # A3A4="driving_c"
    # A4A5="driving_f"
    # A5A6="driving_h"
    # A6A7="driving_i"
    # A7A8="driving_j"

    A1A2="stanley_a"
    A2A3="parking_b"
    A3A4="curve_c"
    A4A5="driving_f"
    A5A6 = "driving_U"
    B1B2="uturn_g"
    A6A7="stanley_h"
    A7A8="driving_i"
    A8A9="obstacle_j"
    

    ## 이거 아님 다시 생각
    
    # Bunsudae Path
    # A1A2 = "driving_a"  # 13
    # A2A3 = "pickup_b"  # 8
    # A3A4 = "curve_c"  # 13
    # A4A5 = "driving_d"  # 20
    # A5A6 = "curve_e"  # 13
    # A6A7 = "driving_f"  # 20
    # A7A8 = "curve_g"  # 13
    # A8A9 = "driving_h"  # 20
    # A9A10 = "driving_i"  # 20


    # # Bunsudae Path
    # A1A2 = "driving_a"  # 13
    # A2A3 = "pickup_b"  # 8
    # A3A4 = "curve_c"  # 8
    # A4A5 = "driving_d"  # 13
    # A5A6 = "pickup_e"  # 8
    # A6A7 = "driving_f"  # 13
    # A7A8 = "curve_g"  # 8
    # A8A9 = "driving_h"  # 13
    # # A9A10 = "delivery_i"   # 8
    # A9A10 = "driving_i"  # 8
    # A10A11 = "curve_j"  # 8
    # # A11A12 = "driving_k"   # 13
    # A11A12 = "parking_l"  # 8
    # A12A13 = "driving_m"  # 13
    # # Bunsudae Path


class GetPath:
    def __init__(self, db, init_state):
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.cv = []
        self.db = db

        self.file_open_with_id(init_state.name)

    def file_open_with_id(self, id):
        self.cx, self.cy, self.cyaw, self.cv = self.db.query_from_id(id)


class GetOdometry:
    def __init__(self, node, odom_topic):
        self.x = 0.0  # m
        self.y = 0.0  # m
        self.yaw = 0.0  # rad
        self.v = 0.0  # m/s

        self.node = node

        self.node.create_subscription(
            Odometry, odom_topic, self.callback, qos_profile=qos_profile_system_default
        )

    def callback(self, msg):
        self.pose = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.yaw = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )
        self.v = msg.twist.twist.linear.x


class StateMachine:
    def __init__(self, node, odometry, path, state, db, db_mpc):


        self.pub = node.create_publisher(
            ControlMessage, "cmd_msg", qos_profile=qos_profile_system_default
        )
        self.path_pub = node.create_publisher(
            Path, "global_path", qos_profile=qos_profile_system_default
        )
        node.create_subscription(
            Float32, "hdr", self.hdr_callback, qos_profile=qos_profile_system_default
        )
        node.create_subscription(
            Float32, "ctr", self.ctr_callback, qos_profile=qos_profile_system_default
        )
        self.publish_mode = node.create_publisher(
            String, "jamming_status", qos_profile=qos_profile_system_default)

        self.db = db
        self.db_mpc = db_mpc

        self.jamming_status = "False"

        self.node = node
        self.state = state
        self.path = path
        self.odometry = odometry

        self.st = Stanley()
        self.mpc = MPC(self.db_mpc)
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)
        self.pc = ParamConfigurer(node)
        # self.lc = LifecycleController(node)

        self.target_idx = 0
        self.mission_finish = False

        self.obstacle = Obstacle(self.node)
        self.uturn = Uturn(self.node)
        self.parking = Parking(self.node)

        self.first_flags = {s: True for s in ["driving","parking","uturn","obstacle", "obstacle_jamming"]}

        self.min = 0
        self.max = 25
        self.hdr = 0.0
        self.ctr = 0.0
        
        self.publish_mode.publish(msg = String(data = self.jamming_status))

    def update_state_and_path(self):
        if self.state.value[:-2] in ["driving","curve","stanley"]:
            print(f"{'-'*37}\n{self.target_idx}  /  {len(self.path.cx)}\n{'-'*37}")
            if self.target_idx >= len(self.path.cx) - 10:  # driving에서 state 전환 조건
                states = list(State)
                current_index = states.index(self.state)
                try:
                    self.first_flags["diriving"] = True
                    self.state = states[
                        current_index + 1
                    ]  # state update (driving -> mission)
                    self.target_idx = 0  # target_idx reset
                    self.path.file_open_with_id(self.state.name)  # path update
                    self.publish_path()  # path publish
                    self.mission_finish = False
                except IndexError:
                    print("index out of range")
        else:
            print(f"{'-'*37}\n{self.target_idx}  /  {len(self.path.cx)}\n{'-'*37}")
            if self.mission_finish:  # mission에서 state 전환 조건
                states = list(State)
                current_index = states.index(self.state)
                try:
                    self.state = states[
                        current_index + 1
                    ]  # state update (mission -> driving)
                    self.path.file_open_with_id(self.state.name)  # path update
                    self.publish_path()  # path publish
                except IndexError:
                    print("index out of range")

    def update_cmd_msg(self):
        print(self.state.value)
        # if self.state.value == "driving_i":
        #     self.activate_ndt(state="driving_i")
        msg = ControlMessage()
        steer, speed_output = self.mpc.pose_callback(self.odometry.pose)
        print(f"steer: {steer}, speed: {speed_output}")
        if self.state.value[:-2] == "driving" or self.state.value[:-2] == "curve":
            self.activate_detection_area(state="driving",area = [0.0, 0.0, 0.0, 0.0])
            msg = self.control_mpc(speed_output, steer)

        elif self.state.value[:-2] == "stanley":
            self.activate_detection_area(state="driving",area = [0.0, 0.0, 0.0, 0.0])
            msg = self.control_stanley()
            
        elif self.state.value[:-2] == "obstacle":
            self.activate_detection_area(state = "obstacle", area = [0.,5.,-2.5,2.5])
            self.activate_gpa_jamming_mode(state = "obstacle_jamming")
            msg, self.mission_finish = self.obstacle.control_obstacle(self.odometry, self.path)

        elif self.state.value[:-2] == "uturn":
            self.activate_detection_area(state="uturn", area=[0.,10.,-7.,0.5])
            if self.odometry.x != 0.0:
                msg, self.mission_finish = self.uturn.control_uturn(self.odometry)
        
        elif self.state.value[:-2] == "parking":
            self.activate_detection_area(state="parking", area=[-2.0, 8.0, -8.0, 0.5])
            if self.odometry.x != 0.0:
                msg, self.mission_finish = self.parking.control_parking(self.odometry)
        else:
            print("error: ", self.state.value)
        return msg
    
    def hdr_callback(self, msg):
        self.hdr = msg.data

    def ctr_callback(self, msg):
        self.ctr = msg.data

    def idx_calc(self):
        idx = self.db.find_idx(self.odometry.x, self.odometry.y, table="path")
        return idx


    def activate_detection_area(self,state,area=[0.0, 0.0, 0.0, 0.0]):
        if self.first_flags.get(state, False):
            self.first_flags[state] = False
            self.pc.set_detection_area(area)

    def activate_gpa_jamming_mode(self,state):
        if self.first_flags.get(state, False):
            self.first_flags[state] = False
            self.jamming_status = "True"
            self.publish_mode.publish(msg = String(data = self.jamming_status))  # path publish
            #self.pc.set_gps_jamming(bool)
    
    # def activate_ndt(self, state):
    #     if self.first_flags.get(state, False):
    #         self.first_flags[state] = False
    #         self.lc.activate_node()

    def control_mpc(self, speed_output, steer):
        msg = ControlMessage()
        if self.odometry.x != 0.0:  # 24.10.03 수정
            self.target_idx, _ = self.st.calc_target_index(self.odometry, self.path.cx, self.path.cy)
            kph_speed = speed_output * 3.6  
            if self.hdr and self.ctr: # if QP cannot solve 
                adapted_speed = self.ss.adaptSpeed(
                    kph_speed,
                    self.hdr,
                    self.ctr,
                    min_value=2,
                    max_value=self.max,
                )
                brake = self.cacluate_brake(adapted_speed)
                msg.speed = int(adapted_speed) * 10
                msg.steer = int(m.degrees((-1) * steer) * 1e3)
                msg.gear = 2
                msg.brake = int(brake)
            else:
                brake = self.cacluate_brake(kph_speed)
                msg.speed = int(kph_speed) * 10
                msg.steer = int(m.degrees((-1) * steer) * 1e3)
                msg.gear = 2
                msg.brake = int(brake)
        else:
            # wait for odom
            pass
        return msg
    
    def control_stanley(self):
        msg = ControlMessage()
        if self.odometry.x != 0.0: 
            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                    self.odometry,
                    self.path.cx,
                    self.path.cy,
                    self.path.cyaw,
                    h_gain=0.5,
                    c_gain=0.24,
                )
            target_speed = self.set_target_speed()
            # 에러(hdr, ctr) 기반 목표 속력 조정
            adapted_speed = self.ss.adaptSpeed(
                target_speed, hdr, ctr, min_value=10, max_value=21
            )  
            # speed 조정 (PI control)
            speed = self.pid.PIDControl(
                self.odometry.v * 3.6, adapted_speed, min=0, max=25
            )  
            # brake 조정
            brake = self.cacluate_brake(adapted_speed)  
            msg.speed = int(speed) * 10
            msg.steer = int(m.degrees((-1) * steer) * 1e3)
            msg.gear = 2
            msg.brake = int(brake)
        return msg
    
    def set_target_speed(self):
        target_speed = self.path.cv[self.target_idx]
        return target_speed

    def cacluate_brake(
        self, adapted_speed
    ):  # brake 값 정하는 알고리즘 좀 더 정교하게 생각
        if self.odometry.v * 3.6 >= adapted_speed:
            brake = (abs(self.odometry.v * 3.6 - adapted_speed) / 20.0) * 200
            brake = np.clip(brake, 0, 200)
        else:
            brake = 0
        return brake

    def publish_cmd(self):
        self.update_state_and_path()
        msg = self.update_cmd_msg()
        self.pub.publish(msg)

    def publish_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        path_msg.header.frame_id = (
            "map"  # Assuming frame_id is 'map', adjust as necessary
        )

        for x, y, yaw in zip(self.path.cx, self.path.cy, self.path.cyaw):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = (
                0.0  # Assuming the path is on the ground, adjust if needed
            )
            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


def main():
    rclpy.init(args=None)
    node = rclpy.create_node("state_machine_node")
    node.declare_parameter("file_name", "YS/kcity_6th_ys_v1" ".db")
    node.declare_parameter("file_name_mpc", "YS/MPC_kcity_6th_ys_v1" ".db")
    # node.declare_parameter("odom_topic", "/localization/kinematic_state")
    node.declare_parameter("odom_topic", "/localization/kinematic_state/rotated")

    # Get Params
    file_name = node.get_parameter("file_name").get_parameter_value().string_value
    file_name_mpc = node.get_parameter("file_name_mpc").get_parameter_value().string_value
    odom_topic = node.get_parameter("odom_topic").get_parameter_value().string_value

    # Declare Instance
    db = DB(file_name)
    db_mpc = DB(file_name_mpc)
    state = State.A1A2
    path = GetPath(db, state)
    odometry = GetOdometry(node, odom_topic)
    state_machine = StateMachine(node, odometry, path, state, db, db_mpc)
    state_machine.publish_path()  # A1A2(초기 path) publish

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    rate = node.create_rate(10)

    while rclpy.ok():
        try:
            state_machine.publish_cmd()
        except Exception as ex:
            print(ex)
        rate.sleep()


if __name__ == "__main__":
    main()
