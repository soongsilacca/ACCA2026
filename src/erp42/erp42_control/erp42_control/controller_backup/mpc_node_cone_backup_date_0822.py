#!/usr/bin/env python3
import math
import numpy as np
from dataclasses import dataclass, field
import cvxpy
from scipy.linalg import block_diag
from scipy.sparse import block_diag, csc_matrix, diags
from scipy.spatial import transform
import uuid
from enum import Enum
from DB import DB
import json

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from stanley import Stanley
from std_msgs.msg import Float32, String
from erp42_msgs.msg import ControlMessage

@dataclass
class mpc_config:
    NXK: int = 4  # length of kinematic state vector: z = [x, y, v, yaw]
    NU: int = 2  # length of input vector: u = [steering speed, acceleration]
    TK: int = 12  # finite time horizon length - kinematic
    # # ---------------------------------------------------
    # 8kph
    # Rk: list = field(
    #     # default_factory=lambda: np.diag([0.01, 100.0])
    #     default_factory=lambda: np.diag([0.5, 70.0])
    #     # default_factory=lambda: np.diag([0.5, 90.0])
    # )  # input cost matrix, penalty for inputs - [accel, steering_speed]
    # Rdk: list = field(
    #     # default_factory=lambda: np.diag([0.01, 100.0])
    #     default_factory=lambda: np.diag([0.5, 300.0])
    #     # default_factory=lambda: np.diag([0.5, 400.0])
    # )  # input difference cost matrix, penalty for change of inputs - [accel, steering_speed]

    # Qk: list = field(
    #     # default_factory=lambda: np.diag([50., 50., 5.5, 13.0])
    #     default_factory=lambda: np.diag([12.0, 12.0, 20.0, 30.0])
    #     # default_factory=lambda: np.diag([12.0, 12.0, 20.0, 30.0])
    #     # (x, y, v, yaw)
    # )
    # Qfk: list = field(
    #     # default_factory=lambda: np.diag([50., 50., 5.5, 13.0])
    #     default_factory=lambda: np.diag([12.0, 12.0, 20.0, 30.0])
    #     # default_factory=lambda: np.diag([12.0, 12.0, 20.0, 30.0])
    #     # final state error matrix, penalty  for the final state constraints: (x, y, v, yaw)
    # )
    # ---------------------------------------------------
    Rk: list = field(
        # default_factory=lambda: np.diag([0.01, 100.0])
        default_factory=lambda: np.diag([0.5, 70.0])
        # default_factory=lambda: np.diag([0.5, 90.0])
    )  # input cost matrix, penalty for inputs - [accel, steering_speed]
    Rdk: list = field(
        # default_factory=lambda: np.diag([0.01, 100.0])
        default_factory=lambda: np.diag([0.5, 450.0])
        # default_factory=lambda: np.diag([0.5, 400.0])
    )  # input difference cost matrix, penalty for change of inputs - [accel, steering_speed]

    Qk: list = field(
        # default_factory=lambda: np.diag([50., 50., 5.5, 13.0])
        default_factory=lambda: np.diag([12.0, 12.0, 20.0, 30.0])
        # default_factory=lambda: np.diag([12.0, 12.0, 20.0, 30.0])
        # (x, y, v, yaw)
    )
    Qfk: list = field(
        # default_factory=lambda: np.diag([50., 50., 5.5, 13.0])
        default_factory=lambda: np.diag([12.0, 12.0, 20.0, 30.0])
        # default_factory=lambda: np.diag([12.0, 12.0, 20.0, 30.0])
        # final state error matrix, penalty  for the final state constraints: (x, y, v, yaw)
    )
    # ---------------------------------------------------
    DTK: float = 0.1  # time step [s] kinematic
    WIDTH: float = 1.160  # Width of the vehicle [m]
    WB: float = 1.040  # Wheelbase [m]
    MIN_STEER: float = -0.4189  # maximum steering angle [rad]
    MAX_STEER: float = 0.4189  # maximum steering angle [rad] # expand
    # MAX_DSTEER = np.deg2rad(38.0)  # 1.05 rad/s
    MAX_DSTEER = np.deg2rad(20.0)
    # MAX_SPEED: float = 6.94  # maximum speed [m/s] ~ 5.0 for levine sim
    # MIN_SPEED: float = 0.0  # minimum backward speed [m/s]
    MAX_SPEED: float = 8.0  
    MIN_SPEED: float = -2.0  
    MAX_ACCEL: float = 10.0  # maximum acceleration [m/ss]
    # dlk: float = 0.25  # dist step [m] kinematic


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    delta: float = 0.0
    v: float = 0.0
    yaw: float = 0.0
    yawrate: float = 0.0
    beta: float = 0.0


class MPC(Node):
    """
    Implement Kinematic MPC on the car
    """

    def __init__(self, db):
        super().__init__(f"mpc_node_{uuid.uuid4().int % 100000}")

        self.declare_parameter("tau_vel", 1.0)  # velocity time constant[s] [1.0 ~ 2.5]
        self.declare_parameter(
            "tau_steer", 0.17
        )  # steering time constant[s] [0.17 ~ 0.4]
        self.declare_parameter("dind", 3)  # distance step [10^-1 m] [3 ~ 5]
        self.declare_parameter("use_latency_model", False)  # use latency model

        self.tau_vel = self.get_parameter("tau_vel").get_parameter_value().double_value
        self.tau_steer = (
            self.get_parameter("tau_steer").get_parameter_value().double_value
        )
        self.dind = self.get_parameter("dind").get_parameter_value().integer_value
        self.use_latency_model = (
            self.get_parameter("use_latency_model").get_parameter_value().bool_value
        )

        self.db = db
        self.config = mpc_config()
        self.st = Stanley()
        self.reset_ws = False  # reset warm start option if state changes
        self.waypoints = self.file_open_with_id()
        self.waypoints = np.array(self.waypoints)
        self.waypoints[3, :] = self.waypoints[3, :] / 3.6  # kph → m/s 0609 modified
        self.odelta_v = None
        self.odelta = None
        self.oa = None


        vis_ref_traj_topic = "/ref_traj_marker"
        vis_waypoints_topic = "/waypoints_marker"
        vis_pred_path_topic = "/pred_path_marker"
        self.vis_waypoints_pub = self.create_publisher(Marker, vis_waypoints_topic, 1)
        self.vis_waypoints_msg = Marker()
        self.vis_ref_traj_pub = self.create_publisher(Marker, vis_ref_traj_topic, 1)
        self.vis_ref_traj_msg = Marker()
        self.vis_pred_path_pub = self.create_publisher(Marker, vis_pred_path_topic, 1)
        self.vis_pred_path_msg = Marker()
        self.pub_hdr = self.create_publisher(Float32, "hdr", 1)
        self.pub_ctr = self.create_publisher(Float32, "ctr", 1)
        self.info_pub = self.create_publisher(String, "/mpc/info", 1)
        self.pub_error = self.create_publisher(String, "/mpc/error", 1)

        self.visualize_waypoints_in_rviz()
        self.mpc_prob_init()

        # publish mpc parameters
        self.publish_startup_info()

    def file_open_with_id(self):
        return self.db.query_from_id("A1A2")

    def pose_callback(self, pose_msg):

        self.vehicle_state = self.update_vehicle_state(pose_msg)
        self.get_logger().info(f"mpc/-----waypoints : {len(self.waypoints[0, : ])}-----")

        self.ref_path, self.target_idx = self.calc_ref_trajectory(
            self.vehicle_state,
            self.waypoints[0, :],
            self.waypoints[1, :],
            self.waypoints[2, :],
            self.waypoints[3, :],
        )
        

        self.visualize_ref_traj_in_rviz(self.ref_path)

        self.visualize_waypoints_in_rviz()

        x0 = [
            self.vehicle_state.x,
            self.vehicle_state.y,
            self.vehicle_state.v,
            self.vehicle_state.yaw,
        ]

        # solve the MPC control problem
        ########################################## 연산 오래걸림 ##########################################
        result = self.linear_mpc_control(self.ref_path, x0, self.oa, self.odelta_v)
        ########################################## 연산 오래걸림 ##########################################

        if result[0] is None:
            print("--------------------------stanly-----------------------------------")

            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                self.vehicle_state,
                self.waypoints[0, :],
                self.waypoints[1, :],
                self.waypoints[2, :],
                h_gain=0.5,
                c_gain=0.24,
            )
            self.pub_hdr.publish(Float32(data=hdr))
            self.pub_ctr.publish(Float32(data=ctr))
            target_speed = self.waypoints[
                3, self.target_idx
            ]  
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

            steer_output = self.odelta_v[0]
            speed_output = self.vehicle_state.v + self.oa[0] * self.config.DTK

            self.pub_hdr.publish(Float32(data=0.0))
            self.pub_ctr.publish(Float32(data=0.0))

        return self.target_idx, steer_output, speed_output

    def update_vehicle_state(self, pose_msg):
        """
        Update the vehicle state from Localization.
        """
        vehicle_state = State()
        vehicle_state.x = pose_msg.pose.pose.position.x
        vehicle_state.y = pose_msg.pose.pose.position.y
        vehicle_state.v = pose_msg.twist.twist.linear.x  # 0812 수정
        curr_orien = pose_msg.pose.pose.orientation
        q = [curr_orien.x, curr_orien.y, curr_orien.z, curr_orien.w]
        vehicle_state.yaw = math.atan2(
            2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)
        )

        ######## skip the time step 1m #########
        # vehicle_state.x = vehicle_state.x + 1.0 * math.cos(vehicle_state.yaw)
        # vehicle_state.y = vehicle_state.y + 1.0 * math.sin(vehicle_state.yaw)

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
        #       self.xk, self.x0k, self.config.MAX_SPEED, self.config.MIN_SPEED,
        #       self.uk, self.config.MAX_ACCEL, self.config.MAX_STEER

        # init state constraint
        c3 = self.xk[:, 0] == self.x0k
        constraints.append(c3)

        # state consraints
        speed = self.xk[2, :]
        c4_lower = self.config.MIN_SPEED <= speed
        c4_upper = speed <= self.config.MAX_SPEED
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

        # Find nearest index from where the trajectories are calculated
        _, _, _, ind = self.nearest_point(
            np.array([state.x, state.y]), np.array([cx, cy]).T
        )

        # Load the initial parameters from the nearest idx into the trajectory
        ref_traj[0, 0] = cx[ind]
        ref_traj[1, 0] = cy[ind]
        ref_traj[2, 0] = sp[ind]
        ref_traj[3, 0] = cyaw[ind]

        dind = self.dind  # distance step

        ind_list = int(ind) + np.insert(
            np.cumsum(np.repeat(dind, self.config.TK)), 0, 0
        ).astype(int)

        ind_list[ind_list >= ncourse] -= ncourse

        ref_traj[0, :] = cx[ind_list]
        ref_traj[1, :] = cy[ind_list]
        ref_traj[2, :] = sp[ind_list]

        angle_thres = 4.5

        for i in range(len(cyaw)):
            if cyaw[i] - state.yaw > angle_thres:
                cyaw[i] -= 2 * np.pi
            if state.yaw - cyaw[i] > angle_thres:
                cyaw[i] += 2 * np.pi

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

    def update_state(self, state, a_cmd, delta_cmd):
        if self.use_latency_model:
            # 지연 상수
            tau_vel = self.tau_vel  # velocity time constant [s]
            tau_steer = self.tau_steer  # steering time constant [s] must 0.17 ~ 0.4
            # tau_steer = 0.17  # steering time constant [s]
            dt = self.config.DTK

            v_next = state.v + (dt / tau_vel) * ((state.v + a_cmd * dt) - state.v)
            delta_next = state.delta + (dt / tau_steer) * (delta_cmd - state.delta)

            # input check
            if delta_next >= self.config.MAX_STEER:
                delta_next = self.config.MAX_STEER
            elif delta_next <= -self.config.MAX_STEER:
                delta_next = -self.config.MAX_STEER

            state.x += v_next * math.cos(state.yaw) * dt
            state.y += v_next * math.sin(state.yaw) * dt
            state.yaw += (v_next / self.config.WB) * math.tan(delta_next) * dt
            state.v = v_next
            state.delta = delta_next
        else:
            if delta_cmd >= self.config.MAX_STEER:
                delta_cmd = self.config.MAX_STEER
            elif delta_cmd <= -self.config.MAX_STEER:
                delta_cmd = -self.config.MAX_STEER

            state.x = state.x + state.v * math.cos(state.yaw) * self.config.DTK
            state.y = state.y + state.v * math.sin(state.yaw) * self.config.DTK
            state.yaw = (
                state.yaw
                + (state.v / self.config.WB) * math.tan(delta_cmd) * self.config.DTK
            )
            state.v = state.v + a_cmd * self.config.DTK

        # 속도 제한
        if state.v > self.config.MAX_SPEED:
            state.v = self.config.MAX_SPEED
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

        Calc linear and discrete time dynamic model with first-order delay
        for steering and velocity.
        State vector: x=[x, y, v, yaw]
        Input vector: u=[accel_cmd, steer_cmd]
        """
        if self.use_latency_model:
            # 차량 지연 상수 (초 단위)
            tau_vel = self.tau_vel  # velocity time constant
            tau_steer = self.tau_steer  # steering time constant

            dt = self.config.DTK  # time step

            # State (or system) matrix A, 4x4
            A = np.zeros((self.config.NXK, self.config.NXK))
            A[0, 0] = 1.0
            A[1, 1] = 1.0
            A[2, 2] = 1.0 - dt / tau_vel  # velocity update with delay
            A[3, 3] = 1.0
            A[0, 2] = self.config.DTK * math.cos(phi)
            A[0, 3] = -self.config.DTK * v * math.sin(phi)
            A[1, 2] = self.config.DTK * math.sin(phi)
            A[1, 3] = self.config.DTK * v * math.cos(phi)
            A[3, 2] = self.config.DTK * math.tan(delta) / self.config.WB

            # Input Matrix B; 4x2
            B = np.zeros((self.config.NXK, self.config.NU))
            # 속도: accel_cmd에 대한 반응
            B[2, 0] = dt / tau_vel
            # 조향: steer_cmd에 대한 반응 (조향 속도가 아니라 조향 명령을 따르도록)
            B[3, 1] = (dt / tau_steer) * (v / (self.config.WB * math.cos(delta) ** 2))
        else:
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
            # self.MPC_prob.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)
            # default max_iter = 4000 eps_abs = 1e-3 eps_rel = 1e-3
            # light solver settings max_iter = 1500, eps_abs = 3e-3, eps_rel = 3e-3
            self.MPC_prob.solve(
                solver=cvxpy.OSQP, verbose=False, warm_start=not self.reset_ws
            )
            if self.reset_ws:
                self.reset_ws = False
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
            self.pub_error.publish(String(data="MPC solve failed"))
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

        self.visualize_pred_path_in_rviz(path_predict)

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

        self.vis_waypoints_pub.publish(self.vis_waypoints_msg)

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

    def publish_startup_info(self):
        """노드 시작 시 분석용 설정 정보를 JSON으로 퍼블리시."""
        try:
            info = {
                "node": self.get_name(),
                "TK": int(self.config.TK),
                "DTK": float(self.config.DTK),
                "weights": {
                    "Rk": np.array(self.config.Rk).tolist(),
                    "Rdk": np.array(self.config.Rdk).tolist(),
                    "Qk": np.array(self.config.Qk).tolist(),
                    "Qfk": np.array(self.config.Qfk).tolist(),
                },
                "vehicle": {
                    "WB": float(self.config.WB),
                    "WIDTH": float(self.config.WIDTH),
                    "STEER_MIN": float(self.config.MIN_STEER),
                    "STEER_MAX": float(self.config.MAX_STEER),
                    "MAX_DSTEER": float(self.config.MAX_DSTEER),
                    "SPEED_MAX": float(self.config.MAX_SPEED),
                    "SPEED_MIN": float(self.config.MIN_SPEED),
                    "ACCEL_MAX": float(self.config.MAX_ACCEL),
                },
                # 런타임 파라미터(런치/파라미터 파일에서 주입 가능)
                "control_time_constants": {
                    "tau_vel": self.get_parameter("tau_vel")
                    .get_parameter_value()
                    .double_value,
                    "tau_steer": self.get_parameter("tau_steer")
                    .get_parameter_value()
                    .double_value,
                    "dind": self.get_parameter("dind")
                    .get_parameter_value()
                    .integer_value,
                },
                "waypoints_meta": {
                    "count": int(self.waypoints.shape[1]),
                    "speed_unit": "m/s",  # 이미 m/s로 변환됨
                },
            }

            msg = String()
            msg.data = json.dumps(info, ensure_ascii=False, separators=(",", ":"))
            self.info_pub.publish(msg)
            self.get_logger().info("[MPC] Startup info published to /mpc/info")

        except Exception as e:
            self.get_logger().error(f"[MPC] Failed to publish startup info: {e}")


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
