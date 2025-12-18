#!/usr/bin/env python3
import math
import numpy as np
from dataclasses import dataclass, field
import cvxpy
from scipy.linalg import block_diag
from scipy.sparse import block_diag, csc_matrix, diags
from scipy.spatial import transform
import os
import uuid
from enum import Enum

import math as m

from DB import DB

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from erp42_msgs.msg import ControlMessage
#, AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from stanley import Stanley
from std_msgs.msg import Float32


####################################################################################
# TODO: not solve fallback controller
####################################################################################


class State_mpc(Enum):
    # kcity 본선 대회용 (final - 1012)
    # A1A2 = "driving_a"  # 13
    # A2A3 = "pickup_b"  # 9
    # A3A4 = "curve_c"  # 8
    # A4A5 = "curve_d"  # 8
    # A5A6 = "obstacle_e"  # 6
    # A6A7 = "curve_f"  # 8
    # A7A8 = "stop_line_a"  # 8
    # A8A9 = "stop_line_b"  # 8
    # A9A10 = "curve_h"  # 8
    # A10A11 = "traffic_light_i"  # 8
    # A11A12 = "curve_j"  # 8
    # A12A13 = "traffic_light_k"  # 8
    # A13A14 = "driving_l"  # 15
    # A14A15 = "obstacle_m"  # 6
    # A15A16 = "curve_n"  # 8
    # A16A17 = "traffic_light_o"  # 8
    # A17A18 = "driving_p"  # 10
    # A18A19 = "delivery_q"  # 7 #delivery
    # A19A20 = "driving_r"  # 8
    # A20A21 = "traffic_light_s"  # 8
    # A21A22 = "driving_t"  # 10
    # A22A23 = "traffic_light_u"  # 8
    # A23A24 = "curve_v"  # 10
    # A24A25 = "driving_w"  # 15
    # A25A26 = "curve_x"  # 11
    # A26A27 = "stop_line_c"  # 8
    # A27A28 = "curve_y"  # 8
    # A28A29 = "driving_z"  # 13
    # A29A30 = "traffic_light_A"  # 8
    # A30A31 = "driving_B"  # 16
    # A31A32 = "traffic_light_C"  # 8
    # A32A33 = "driving_D"  # 15
    # A33A34 = "parking_E"  # 6
    # A34A35 = "driving_E"  # 15

    A1A2 = "driving_a"  # 13
    A2A3 = "driving_b"  # 9
    A3A4 = "driving_c"  # 8
    A4A5 = "driving_d"  # 8
    A5A6 = "driving_e"  # 6
    A6A7 = "driving_f"  # 8
    A7A8 = "driving_g"  # 8
    A8A9 = "driving_s"  # 8
    A9A10 = "driving_h"  # 8


@dataclass
class mpc_config:
    NXK: int = 4  # length of kinematic state vector: z = [x, y, v, yaw]
    NU: int = 2  # length of input vector: u = [steering speed, acceleration]
    TK: int = 20  # finite time horizon length - kinematic

    # ---------------------------------------------------
    # TODO: you may need to tune the following matrices
    Rk: list = field(
        # default_factory=lambda: np.diag([0.01, 100.0])
        default_factory=lambda: np.diag([0.5, 60.0])
    )  # input cost matrix, penalty for inputs - [accel, steering_speed]
    Rdk: list = field(
        # default_factory=lambda: np.diag([0.01, 100.0])
        default_factory=lambda: np.diag([0.5, 60.0])
    )  # input difference cost matrix, penalty for change of inputs - [accel, steering_speed]
    Qk: list = field(
        default_factory=lambda: np.diag([12.0, 12.0, 5.0, 30.0])  # levine sim
        # default_factory=lambda: np.diag([50., 50., 5.5, 13.0])
    )  # state error cost matrix, for the the next (T) prediction time steps [x, y, delta, v, yaw, yaw-rate, beta]
    Qfk: list = field(
        default_factory=lambda: np.diag([12.0, 12.0, 5.0, 30.0])  # levine sim
                                        # (x, y, v, yaw)
        # default_factory=lambda: np.diag([50., 50., 5.5, 13.0])
    )  # final state error matrix, penalty  for the final state constraints: [x, y, delta, v, yaw, yaw-rate, beta]
    # ---------------------------------------------------

    N_IND_SEARCH: int = 10  # Search index number
    DTK: float = 0.1  # time step [s] kinematic
    dlk: float = 0.03  # dist step [m] kinematic
    LENGTH: float = 2.020  # Length of the vehicle [m]
    WIDTH: float = 1.160  # Width of the vehicle [m]
    WB: float = 1.040  # Wheelbase [m]
    MIN_STEER: float = -0.4189  # maximum steering angle [rad]
    MAX_STEER: float = 0.4189  # maximum steering angle [rad] # expand
    MAX_DSTEER = np.deg2rad(60.0)  # 1.05 rad/s
    MAX_STEER_V: float = 3.2  # maximum steering speed [rad/s]
    MIN_SPEED: float = -2.0  # minimum backward speed [m/s]
    MAX_ACCEL: float = 30.0  # maximum acceleration [m/ss]


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    delta: float = 0.0
    v: float = 0.0
    yaw: float = 0.0
    yawrate: float = 0.0
    beta: float = 0.0


class PID:
    def __init__(self, node):
        self.node = node
        self.p_gain = self.node.declare_parameter("/stanley_controller/p_gain", 2.07).value
        self.i_gain = self.node.declare_parameter("/stanley_controller/i_gain", 0.85).value

        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0

        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )

    def PIDControl(self, speed, desired_value, min_val, max_val):

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
        speed = int(np.clip(self.speed, min_val, max_val))

        return speed


class MPC(Node):
    """
    Implement Kinematic MPC on the car
    This is just a template, you are free to implement your own node!
    """

    def __init__(self, db):
        super().__init__(f"mpc_node_{uuid.uuid4().int % 100000}")
        # use the MPC as a tracker (similar to pure pursuit)
        self.is_real = False
        # self.map_name = 'levine_2nd'
        self.db = db
        self.max_speed = 0.0
        # print(id.value)
        # a = id.value
        # a = str(a)

        self.state = State_mpc.A1A2

        # create ROS subscribers and publishers
        pose_topic = (
            "/pf/viz/inferred_pose" if self.is_real else "/localization/kinematic_state"
        )
        drive_topic = "/mpc_cmd"
        vis_ref_traj_topic = "/ref_traj_marker"
        vis_waypoints_topic = "/waypoints_marker"
        vis_pred_path_topic = "/pred_path_marker"
        cmd_topic = "/cmd_msg"

        self.pose_sub = self.create_subscription(
            PoseStamped if self.is_real else Odometry, pose_topic, self.pose_callback, 1
        )
        self.pose_sub  # prevent unused variable warning

        # self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 1) # mspeed
        # self.drive_msg = AckermannDriveStamped()

        self.cmd_pub = self.create_publisher(ControlMessage, cmd_topic, 1)
        self.cmd_msg = ControlMessage()

        self.vis_waypoints_pub = self.create_publisher(Marker, vis_waypoints_topic, 1)
        self.vis_waypoints_msg = Marker()
        self.vis_ref_traj_pub = self.create_publisher(Marker, vis_ref_traj_topic, 1)
        self.vis_ref_traj_msg = Marker()
        self.vis_pred_path_pub = self.create_publisher(Marker, vis_pred_path_topic, 1)
        self.vis_pred_path_msg = Marker()
        self.pub_hdr =  self.create_publisher(Float32, "hdr", 1)
        self.pub_ctr = self.create_publisher(Float32, "ctr", 1)

        # map_path = os.path.abspath(os.path.join('src', 'csv_data'))
        # self.waypoints = np.loadtxt(map_path + '/' + self.map_name + '.csv', delimiter=';', skiprows=0)  # csv data
        self.waypoints = self.file_open_with_id(self.state.name)
        self.waypoints = np.array(self.waypoints)
        self.max_speed = (float(self.waypoints[3, 0]) / 3.6) + 1

        self.waypoints[3, :] = float(self.waypoints[3, :]) / 3.6 # 0609수정//kph → m/s
        # self.waypoints = np.array(self.file_open_with_id[:])
        # if self.map_name == 'levine_2nd':
        #     self.waypoints[:, 2] += math.pi / 2
        self.visualize_waypoints_in_rviz()

        self.config = mpc_config()
        self.odelta_v = None
        self.odelta = None
        self.oa = None
        self.init_flag = 0

        self.init_time = self.get_clock().now().seconds_nanoseconds()[0] + (
            self.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        self.time = []

        self.last = self.get_clock().now().seconds_nanoseconds()[0] + (
            self.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )

        self.prev_err = 0.0

        # initialize MPC problem
        self.mpc_prob_init()

        # init state - avoid unknown variables for scan callback
        self.curr_pos = np.array([0.0, 0.0, 0.0])
        self.rot_mat = np.identity(3)

        self.st = Stanley()
        self.odom_msg = Odometry()
        self.pid = PID(self)

        self.__L = 1.240

    def file_open_with_id(self, id):
        return self.db.query_from_id(id)

    def pose_callback(self, pose_msg):#, speed_msg):
        # extract pose from ROS msg
        # self.update_rotation_matrix(pose_msg)
        # self.max_speed = self.waypoints[0, 3]
        self.vehicle_state = self.update_vehicle_state(pose_msg)#, speed_msg)

        if self.is_real:
            self.vehicle_state.v = (
                -1 * self.vehicle_state.v
            )  # negate the monitoring speed

        # self.waypoints가 유효하지 않으면 함수 종료
        # if self.waypoints is None or len(self.waypoints.shape) != 2 or self.waypoints.shape[1] < 4:
        #     self.get_logger().warn("waypoints 데이터가 부족하거나 잘못된 형식입니다.")
        #     return
        # self.get_logger().info(f"waypoints shape: {self.waypoints}")
        # TODO: Calculate the next reference trajectory for the next T steps with current vehicle pose.
        # ref_x, ref_y, ref_yaw, ref_v are columns of self.waypoints
        self.ref_path, self.target_idx = self.calc_ref_trajectory(
            self.vehicle_state,
            self.waypoints[0, :],
            self.waypoints[1, :],
            self.waypoints[2, :],
            self.waypoints[3, :],
        )
        
        self.max_speed = (float(self.waypoints[3, self.target_idx]) / 3.6) + 1

        # self.get_logger().info(f"len : {len(self.ref_path[0, :])}")

        # self.get_logger().info(f"target_idx : {self.target_idx}")

        self.visualize_ref_traj_in_rviz(self.ref_path)

        x0 = [
            self.vehicle_state.x,
            self.vehicle_state.y,
            self.vehicle_state.v,
            self.vehicle_state.yaw,
        ]

        
        # solve the MPC control problem
        
        result = self.linear_mpc_control(self.ref_path, x0, self.oa, self.odelta_v)
        # self.get_logger().info(f'result : {result}')
        

        # self.oa = None
        # self.odelta = None

        if not result:
            print("--------------------------stanly-----------------------------------")

            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                self.vehicle_state,
                self.waypoints[0, :],
                self.waypoints[1, :],
                self.waypoints[2, :],
                h_gain=0.5,
                c_gain=0.24,
            )
            self.pub_hdr.publish(Float32(hdr))
            self.pub_ctr.publish(Float32(ctr))
            target_speed = self.waypoints[3, self.target_idx]
            return self.target_idx, steer, target_speed
        
        elif result[0] is None or result[1] is None:
            print("--------------------------stanly-----------------------------------")

            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                self.vehicle_state,
                self.waypoints[0, :],
                self.waypoints[1, :],
                self.waypoints[2, :],
                h_gain=0.5,
                c_gain=0.24,
            )
            self.pub_hdr.publish(Float32(hdr))
            self.pub_ctr.publish(Float32(ctr))
            target_speed = self.waypoints[3, self.target_idx]
            return self.target_idx, steer, target_speed

        else:
            (
                self.oa,
                self.odelta_v,
                ox,
                oy,
                oyaw,
                ov,
                state_predict,
            ) = result

            # self.get_logger().info(f"speed : {self.vehicle_state.v}")

            # publish drive message.
            steer_output = self.odelta_v[0]
            # print(self.odelta_v[0])
            speed_output = self.vehicle_state.v + self.oa[0] * self.config.DTK

            self.pub_hdr.publish(Float32(0.0))
            self.pub_ctr.publish(Float32(0.0))
            # self.get_logger().info(f"a : {self.oa[0]}")
            # 
            # print(f"입력 speed: {self.vehicle_state.v}")
            # MPC 내부 예측 궤적, 상태값, 제어입력 출력
            # print(f"목표 속도 벡터: {self.ref_traj_k.value[2, :]}")
            # print(f"최적화 결과 가속도 uk[0]: {self.uk.value[0, :]}")
            # 결과 속도값 (speed_output) 출력
            # print(f"pose_callback 최종 speed_output: {speed_output}")


            # self.drive_msg.steering_angle = steer_output
            # self.drive_msg.speed = (-1.0 if self.is_real else 1.0) * speed_output
            # self.drive_pub.publish(self.drive_msg)
            # print("steering ={}, speed ={}".format(self.drive_msg.drive.steering_angle, self.drive_msg.drive.speed))

        self.vis_waypoints_pub.publish(self.vis_waypoints_msg)

        return self.target_idx, steer_output, (-1.0 if self.is_real else 1.0) * speed_output
        

    # toolkits
    def update_rotation_matrix(self, pose_msg):
        # get rotation matrix from the car frame to the world frame
        curr_orien = (
            pose_msg.pose.orientation
            if self.is_real
            else pose_msg.pose.pose.orientation
        )
        quat = [curr_orien.x, curr_orien.y, curr_orien.z, curr_orien.w]
        self.rot_mat = (transform.Rotation.from_quat(quat)).as_matrix()
        # print("rotation matrix = {}".format(self.rot_mat))

    def update_vehicle_state(self, pose_msg): #, speed_msg):
        """
        written by Derek, not from the template, != update state
        """
        vehicle_state = State()
        vehicle_state.x = (
            pose_msg.pose.position.x if self.is_real else pose_msg.pose.pose.position.x
        )
        vehicle_state.y = (
            pose_msg.pose.position.y if self.is_real else pose_msg.pose.pose.position.y
        )
        # vehicle_state.v = self.drive_msg.drive.speed
        # vehicle_state.v = (
        #     speed_msg.speed
        # )
        vehicle_state.v = pose_msg.twist.twist.linear.x  # 0609 수정//실제 속도 반영을 통한 속도 안정화
        # print(vehicle_state.v)
        # self.get_logger().info(f'speed : {vehicle_state.v * 3.6}')

        curr_orien = (
            pose_msg.pose.orientation
            if self.is_real
            else pose_msg.pose.pose.orientation
        )
        q = [curr_orien.x, curr_orien.y, curr_orien.z, curr_orien.w]
        vehicle_state.yaw = math.atan2(
            2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)
        )
        # https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Quaternion_%E2%86%92_Euler_angles_(z-y%E2%80%B2-x%E2%80%B3_intrinsic)
        # print("yaw =", vehicle_state.yaw)

        return vehicle_state

    # mpc functions
    def mpc_prob_init(self):
        """
        Create MPC quadratic optimization problem using cvxpy, solver: OSQP
        Will be solved every iteration for control.
        More MPC problem information here: https://osqp.org/docs/examples/mpc.html
        More QP example in CVXPY here: https://www.cvxpy.org/examples/basic/quadratic_program.html
        """
        # Initialize and create vectors for the optimization problem
        # Vehicle State Vector
        self.xk = cvxpy.Variable((self.config.NXK, self.config.TK + 1))  # 4 x 9
        # Control Input vector
        self.uk = cvxpy.Variable((self.config.NU, self.config.TK))  # 2 x 8
        objective = 0.0  # Objective value of the optimization problem
        constraints = []  # Create constraints array

        # Initialize reference vectors
        self.x0k = cvxpy.Parameter((self.config.NXK,))  # 4
        self.x0k.value = np.zeros((self.config.NXK,))

        # Initialize reference trajectory parameter
        self.ref_traj_k = cvxpy.Parameter(
            (self.config.NXK, self.config.TK + 1)
        )  # 4 x 9
        self.ref_traj_k.value = np.zeros((self.config.NXK, self.config.TK + 1))

        # Initializes block diagonal form of R = [R, R, ..., R] (NU*T, NU*T)
        R_block = block_diag(
            tuple([self.config.Rk] * self.config.TK)
        )  # (2 * 8) x (2 * 8)

        # Initializes block diagonal form of Rd = [Rd, ..., Rd] (NU*(T-1), NU*(T-1))
        Rd_block = block_diag(
            tuple([self.config.Rdk] * (self.config.TK - 1))
        )  # (2 * 7) x (2 * 7)

        # Initializes block diagonal form of Q = [Q, Q, ..., Qf] (NX*T, NX*T)
        Q_block = [self.config.Qk] * (self.config.TK)  # (4 * 8) x (4 * 8)
        Q_block.append(self.config.Qfk)
        Q_block = block_diag(tuple(Q_block))  # (4 * 9) x (4 * 9), Qk + Qfk

        # Formulate and create the finite-horizon optimal control problem (objective function)
        # The FTOCP has the horizon of T timesteps

        # --------------------------------------------------------
        # TODO: fill in the objectives here, you should be using cvxpy.quad_form() somehwhere

        # Objective part 1: Influence of the control inputs: Inputs u multiplied by the penalty R
        objective += cvxpy.quad_form(
            cvxpy.vec(self.uk), R_block
        )  # # cvxpy.vec() - Flattens the matrix X into a vector in column-major order

        # Objective part 2: Deviation of the vehicle from the reference trajectory weighted by Q, including final Timestep T weighted by Qf
        objective += cvxpy.quad_form(cvxpy.vec(self.xk - self.ref_traj_k), Q_block)

        # Objective part 3: Difference from one control input to the next control input weighted by Rd
        objective += cvxpy.quad_form(cvxpy.vec(cvxpy.diff(self.uk, axis=1)), Rd_block)

        # --------------------------------------------------------

        # Constraints 1: Calculate the future vehicle behavior/states based on the vehicle dynamics model matrices
        # Evaluate vehicle Dynamics for next T timesteps
        A_block = []
        B_block = []
        C_block = []
        # init path to zeros
        path_predict = np.zeros((self.config.NXK, self.config.TK + 1))  # 4 x 9
        for t in range(self.config.TK):  # 8
            A, B, C = self.get_model_matrix(
                path_predict[2, t],
                path_predict[3, t],
                0.0,  # reference steering angle is zero
            )
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))  # 32 x 32
        B_block = block_diag(tuple(B_block))  # 32 x 16
        C_block = np.array(C_block)  # 32 x 1
        # creating the format of matrices

        # [AA] Sparse matrix to CVX parameter for proper stuffing
        # Reference: https://github.com/cvxpy/cvxpy/issues/1159#issuecomment-718925710
        m, n = A_block.shape  # 32, 32
        self.Annz_k = cvxpy.Parameter(
            A_block.nnz
        )  # nnz: number of nonzero elements, nnz = 128
        data = np.ones(self.Annz_k.size)  # 128 x 1, size = 128, all elements are 1
        rows = A_block.row * n + A_block.col  # No. ? element in 32 x 32 matrix
        cols = np.arange(
            self.Annz_k.size
        )  # 128 elements that need to be care - diagonal & nonzero, 4 x 4 x 8
        Indexer = csc_matrix(
            (data, (rows, cols)), shape=(m * n, self.Annz_k.size)
        )  # (rows, cols)	data

        # Setting sparse matrix data
        self.Annz_k.value = A_block.data

        # Now we use this sparse version instead of the old A_block matrix
        self.Ak_ = cvxpy.reshape(Indexer @ self.Annz_k, (m, n), order="C")
        # https://www.cvxpy.org/api_reference/cvxpy.atoms.affine.html#cvxpy.reshape

        # Same as A
        m, n = B_block.shape  # 32, 16 = 4 x 8, 2 x 8
        self.Bnnz_k = cvxpy.Parameter(B_block.nnz)  # nnz = 64
        data = np.ones(self.Bnnz_k.size)  # 64 = (4 x 2) x 8
        rows = B_block.row * n + B_block.col  # No. ? element in 32 x 16 matrix
        cols = np.arange(self.Bnnz_k.size)  # 0, 1, ... 63
        Indexer = csc_matrix(
            (data, (rows, cols)), shape=(m * n, self.Bnnz_k.size)
        )  # (rows, cols)	data

        # sparse version instead of the old B_block
        self.Bk_ = cvxpy.reshape(Indexer @ self.Bnnz_k, (m, n), order="C")

        # real data
        self.Bnnz_k.value = B_block.data

        # No need for sparse matrices for C as most values are parameters
        self.Ck_ = cvxpy.Parameter(C_block.shape)
        self.Ck_.value = C_block

        # -------------------------------------------------------------
        # TODO: Constraint part 1:
        #       Add dynamics constraints to the optimization problem
        #       This constraint should be based on a few variables:
        #       self.xk, self.Ak_, self.Bk_, self.uk, and self.Ck_

        flatten_prev_xk = cvxpy.vec(self.xk[:, :-1])
        flatten_next_xk = cvxpy.vec(self.xk[:, 1:])
        # flatten_uk = cvxpy.diag(self.uk[:, :-1].flatten())
        # import pdb; pdb.set_trace()
        c1 = (
            flatten_next_xk
            == self.Ak_ @ flatten_prev_xk + self.Bk_ @ cvxpy.vec(self.uk) + self.Ck_
        )
        constraints.append(c1)

        # TODO: Constraint part 2:
        #       Add constraints on steering, change in steering angle
        #       cannot exceed steering angle speed limit. Should be based on:
        #       self.uk, self.config.MAX_DSTEER, self.config.DTK

        dsteering = cvxpy.diff(self.uk[1, :])
        c2_lower = -self.config.MAX_DSTEER * self.config.DTK <= dsteering
        c2_upper = dsteering <= self.config.MAX_DSTEER * self.config.DTK
        constraints.append(c2_lower)
        constraints.append(c2_upper)

        # TODO: Constraint part 3:
        #       Add constraints on upper and lower bounds of states and inputs
        #       and initial state constraint, should be based on:
        #       self.xk, self.x0k, self.max_speed, self.config.MIN_SPEED,
        #       self.uk, self.config.MAX_ACCEL, self.config.MAX_STEER

        # init state constraint
        c3 = self.xk[:, 0] == self.x0k
        constraints.append(c3)

        # state consraints
        speed = self.xk[2, :]
        c4_lower = self.config.MIN_SPEED <= speed
        c4_upper = speed <= self.max_speed
        constraints.append(c4_lower)
        constraints.append(c4_upper)

        # input constraints
        steering = self.uk[1, :]
        c5_lower = self.config.MIN_STEER <= steering
        c5_upper = steering <= self.config.MAX_STEER
        constraints.append(c5_lower)
        constraints.append(c5_upper)

        acc = self.uk[0, :]
        c6 = acc <= self.config.MAX_ACCEL
        constraints.append(c6)

        # -------------------------------------------------------------

        # Create the optimization problem in CVXPY and setup the workspace
        # Optimization goal: minimize the objective function
        self.MPC_prob = cvxpy.Problem(cvxpy.Minimize(objective), constraints)

    def nearest_point(self, point, trajectory):
        """
        Return the nearest point along the given piecewise linear trajectory.
        Args:
            point (numpy.ndarray, (2, )): (x, y) of current pose
            trajectory (numpy.ndarray, (N, 2)): array of (x, y) trajectory waypoints
                NOTE: points in trajectory must be unique. If they are not unique, a divide by 0 error will destroy the world
        Returns:
            nearest_point (numpy.ndarray, (2, )): nearest point on the trajectory to the point
            nearest_dist (float): distance to the nearest point
            t (float): nearest point's location as a segment between 0 and 1 on the vector formed by the closest two points on the trajectory. (p_i---*-------p_i+1)
            i (int): index of nearest point in the array of trajectory waypoints
        """
        diffs = trajectory[1:, :] - trajectory[:-1, :]
        l2s = diffs[:, 0] ** 2 + diffs[:, 1] ** 2
        dots = np.empty((trajectory.shape[0] - 1,))
        for i in range(dots.shape[0]):
            dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
        t = dots / l2s
        t[t < 0.0] = 0.0
        t[t > 1.0] = 1.0
        projections = trajectory[:-1, :] + (t * diffs.T).T
        dists = np.empty((projections.shape[0],))
        for i in range(dists.shape[0]):
            temp = point - projections[i]
            dists[i] = np.sqrt(np.sum(temp * temp))
        min_dist_segment = np.argmin(dists)

        return (
            projections[min_dist_segment],
            dists[min_dist_segment],
            t[min_dist_segment],
            min_dist_segment,
        )

    def calc_ref_trajectory(self, state, cx, cy, cyaw, sp):
        """
        calc referent trajectory ref_traj in T steps: [x, y, v, yaw]
        using the current velocity, calc the T points along the reference path
        :param cx: Course X-Position
        :param cy: Course y-Position
        :param cyaw: Course Headingtarget_idx
        :param sp: speed profile
        :dl: distance step
        :pind: Setpoint Index
        :return: reference trajectory ref_traj, reference steering angle
        """

        # Create placeholder Arrays for the reference trajectory for T steps
        ref_traj = np.zeros((self.config.NXK, self.config.TK + 1))
        ncourse = len(cx)

        # Find nearest index/setpoint from where the trajectories are calculated
        _, _, _, ind = self.nearest_point(
            np.array([state.x, state.y]), np.array([cx, cy]).T
        )

        # Load the initial parameters from the setpoint into the trajectory
        ref_traj[0, 0] = cx[ind]
        ref_traj[1, 0] = cy[ind]
        ref_traj[2, 0] = sp[ind]
        ref_traj[3, 0] = cyaw[ind]

        # based on current velocity, distance traveled on the ref line between time steps
        # MAX_DIND = 3                   # ❶ 코너 반경·차축 거리로부터 결정
        # travel  = abs(state.v) * self.config.DTK

        # dind = int(np.clip(round(travel / self.config.dlk) + 1, 1, MAX_DIND))
        # self.get_logger().info(f'travel : {dind}')
        dind = 3
        rest_idx_num = max(
            len(cx) - ind - 1, 1
        )  # 남아있는 idx 개수에서 내 위치 뺀 것과 1 중에서 큰 값 결정, 최소 1 확보
        max_dind = int(
            rest_idx_num / self.config.TK
        )  # 남아있는 idx 개수를 예측하고 싶은 horizon으로 나눠 각 스텝 간 최대 간격 계산
        dind = min(
            dind, max(1, max_dind)
        )  # 계산된 최대 간격과 1을 비교해서 가장 큰 값을 결정하고, 그 값을 지정해놓은 간격과 비교해서 가장 작은 값을 결정
        ind_offsets = np.insert(
            np.cumsum([dind] * self.config.TK), 0, 0
        )  # [dind, dind, ... , dind] TK개  [dind, 2*dind, ... , TK*dind]  [0, dind, ... , TK*dind] - 내 위치 추가
        ind_list = np.clip(int(ind) + ind_offsets, 0, len(cx) - 1).astype(
            int
        )  # 현재 idx 기준으로 미래 idx 계산
        ind_list[ind_list >= ncourse] = ncourse - 1  # 마지막 인덱스로 고정

        ref_traj[0, :] = cx[ind_list]
        ref_traj[1, :] = cy[ind_list]
        ref_traj[2, :] = sp[ind_list]
        
        # if ind >= len(cx) - 10:  # driving에서 state 전환 조건
        #     states = list(State_mpc)
        #     current_index = states.index(self.state)
        #     print
        #     try:
        #         self.state = states[
        #             current_index + 1
        #         ]  # state update (driving -> mission)
        #         self.waypoints = self.file_open_with_id(self.state.name)  # path update
        #         self.waypoints[:, 3] = self.waypoints[:, 3] / 3.6
        #     except IndexError:
        #         print("index out of range")

        angle_thres = 4.5
        # https://edstem.org/us/courses/34340/discussion/2817574

        for i in range(len(cyaw)):
            if cyaw[i] - state.yaw > angle_thres:
                cyaw[i] -= 2 * np.pi
                # print(cyaw[i] - state.yaw)
            if state.yaw - cyaw[i] > angle_thres:
                cyaw[i] += 2 * np.pi
                # print(cyaw[i] - state.yaw)

        # cyaw[cyaw - state.yaw > angle_thres] = np.abs(
        #     cyaw[cyaw - state.yaw > angle_thres] - (2 * np.pi)
        # )
        # cyaw[cyaw - state.yaw < -angle_thres] = np.abs(
        #     cyaw[cyaw - state.yaw < -angle_thres] + (2 * np.pi)
        # )
        ref_traj[3, :] = cyaw[ind_list]

        return ref_traj, ind

    def predict_motion(self, x0, oa, od, xref):
        path_predict = xref * 0.0
        for i, _ in enumerate(x0):
            path_predict[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for ai, di, i in zip(oa, od, range(1, self.config.TK + 1)):
            state = self.update_state(state, ai, di)
            path_predict[0, i] = state.x
            path_predict[1, i] = state.y
            path_predict[2, i] = state.v
            path_predict[3, i] = state.yaw

        return path_predict

    def update_state(self, state, a, delta):

        # input check
        if delta >= self.config.MAX_STEER:
            delta = self.config.MAX_STEER
        elif delta <= -self.config.MAX_STEER:
            delta = -self.config.MAX_STEER

        state.x = state.x + state.v * math.cos(state.yaw) * self.config.DTK
        state.y = state.y + state.v * math.sin(state.yaw) * self.config.DTK
        state.yaw = (
            state.yaw + (state.v / self.config.WB) * math.tan(delta) * self.config.DTK
        )
        state.v = state.v + a * self.config.DTK

        if state.v > self.max_speed:
            state.v = self.max_speed
        elif state.v < self.config.MIN_SPEED:
            state.v = self.config.MIN_SPEED

        return state

    def get_model_matrix(self, v, phi, delta):
        """
        Calc linear and discrete time dynamic model-> Explicit discrete time-invariant
        Linear System: Xdot = Ax +Bu + C
        State vector: x=[x, y, v, yaw]
        :param v: speed
        :param phi: heading angle of the vehicle
        :param delta: steering angle: delta_bar
        :return: A, B, C
        """

        # State (or system) matrix A, 4x4
        A = np.zeros((self.config.NXK, self.config.NXK))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.config.DTK * math.cos(phi)
        A[0, 3] = -self.config.DTK * v * math.sin(phi)
        A[1, 2] = self.config.DTK * math.sin(phi)
        A[1, 3] = self.config.DTK * v * math.cos(phi)
        A[3, 2] = self.config.DTK * math.tan(delta) / self.config.WB

        # Input Matrix B; 4x2
        B = np.zeros((self.config.NXK, self.config.NU))
        B[2, 0] = self.config.DTK
        B[3, 1] = self.config.DTK * v / (self.config.WB * math.cos(delta) ** 2)

        C = np.zeros(self.config.NXK)
        C[0] = self.config.DTK * v * math.sin(phi) * phi
        C[1] = -self.config.DTK * v * math.cos(phi) * phi
        C[3] = -self.config.DTK * v * delta / (self.config.WB * math.cos(delta) ** 2)

        return A, B, C  # 4 x 4, 4 x 2, 4 x 1

    def mpc_prob_solve(self, ref_traj, path_predict, x0):
        self.x0k.value = x0

        A_block = []
        B_block = []
        C_block = []
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(path_predict[2, t], path_predict[3, t], 0.0)
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        self.Annz_k.value = A_block.data
        self.Bnnz_k.value = B_block.data
        self.Ck_.value = C_block

        self.ref_traj_k.value = ref_traj

        # Solve the optimization problem in CVXPY
        # Solver selections: cvxpy.OSQP; cvxpy.GUROBI
        try:
            self.MPC_prob.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)
        except Exception as e:
            print(f"[MPC] Solve failed with exception: {e}")
            return None, None, None, None, None, None

        if (
            self.MPC_prob.status == cvxpy.OPTIMAL
            or self.MPC_prob.status == cvxpy.OPTIMAL_INACCURATE
        ):
            ox = np.array(self.xk.value[0, :]).flatten()
            oy = np.array(self.xk.value[1, :]).flatten()
            ov = np.array(self.xk.value[2, :]).flatten()
            oyaw = np.array(self.xk.value[3, :]).flatten()
            oa = np.array(self.uk.value[0, :]).flatten()
            odelta = np.array(self.uk.value[1, :]).flatten()

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov

    def linear_mpc_control(self, ref_path, x0, oa, od):
        """
        MPC control with updating operational point iteraitvely
        :param ref_path: reference trajectory in T steps
        :param x0: initial state vector
        :param oa: acceleration of T steps of last time
        :param od: delta of T steps of last time
        """

        if oa is None or od is None:
            oa = [0.0] * self.config.TK
            od = [0.0] * self.config.TK

        # Call the Motion Prediction function: Predict the vehicle motion for x-steps
        path_predict = self.predict_motion(x0, oa, od, ref_path)
        # sth to be done to fix the path?
        self.visualize_pred_path_in_rviz(path_predict)

        poa, pod = oa[:], od[:]

        # Run the MPC optimization: Create and solve the optimization problem
        mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v = self.mpc_prob_solve(
            ref_path, path_predict, x0
        )

        return mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v, path_predict

    # visualization
    def visualize_waypoints_in_rviz(self):
        self.vis_waypoints_msg.points = []
        self.vis_waypoints_msg.header.frame_id = "/odom"
        self.vis_waypoints_msg.type = Marker.POINTS
        self.vis_waypoints_msg.color.g = 0.75
        self.vis_waypoints_msg.color.a = 1.0
        self.vis_waypoints_msg.scale.x = 0.05
        self.vis_waypoints_msg.scale.y = 0.05
        self.vis_waypoints_msg.id = 0
        for i in range(self.waypoints.shape[0]):
            point = Point(x=self.waypoints[i, 0], y=self.waypoints[i, 1], z=0.1)
            self.vis_waypoints_msg.points.append(point)

        # self.vis_waypoints_pub.publish(self.vis_waypoints_msg)

    def visualize_ref_traj_in_rviz(self, ref_traj):
        # visualize the path data in the world frame
        self.vis_ref_traj_msg.points = []
        self.vis_ref_traj_msg.header.frame_id = "/odom"
        self.vis_ref_traj_msg.type = Marker.LINE_STRIP
        self.vis_ref_traj_msg.color.b = 0.75
        self.vis_ref_traj_msg.color.a = 1.0
        self.vis_ref_traj_msg.scale.x = 0.08
        self.vis_ref_traj_msg.scale.y = 0.08
        self.vis_ref_traj_msg.id = 0
        for i in range(ref_traj.shape[1]):
            point = Point(x=ref_traj[0, i], y=ref_traj[1, i], z=0.2)
            self.vis_ref_traj_msg.points.append(point)

        self.vis_ref_traj_pub.publish(self.vis_ref_traj_msg)

    def visualize_pred_path_in_rviz(self, path_predict):
        # visualize the path data in the world frame
        self.vis_pred_path_msg.points = []
        self.vis_pred_path_msg.header.frame_id = "/odom"
        self.vis_pred_path_msg.type = Marker.LINE_STRIP
        self.vis_pred_path_msg.color.r = 0.75
        self.vis_pred_path_msg.color.a = 1.0
        self.vis_pred_path_msg.scale.x = 0.08
        self.vis_pred_path_msg.scale.y = 0.08
        self.vis_pred_path_msg.id = 0
        for i in range(path_predict.shape[1]):
            point = Point(x=path_predict[0, i], y=path_predict[1, i], z=0.2)
            self.vis_pred_path_msg.points.append(point)

        self.vis_pred_path_pub.publish(self.vis_pred_path_msg)


def main(args=None):
    file_name = "mpc_test_0520.db"
    db = DB(file_name)

    rclpy.init(args=args)
    print("MPC Initialized")
    mpc_node = MPC(db)
    rclpy.spin(mpc_node)

    mpc_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
