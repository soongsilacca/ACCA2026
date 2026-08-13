#!/usr/bin/env python3
from collections import deque
import math
import os
from pathlib import PosixPath
import sys
import threading

import message_filters
import numpy as np
import rospy
import torch
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CompressedImage, Image, Imu, NavSatFix, PointCloud2
from std_msgs.msg import Float32, Float32MultiArray

from multimodal_learning.bag_tokens import mgeo_tokens_from_msg, route_tokens_from_msg
from multimodal_learning.io_utils import load_yaml, package_config
from multimodal_learning.lidar_bev import pointcloud2_to_xyz, points_to_bev
from multimodal_learning.sensor_preprocess import image_from_msg


class InferenceNode:
    def __init__(self):
        self.topics = load_yaml(rospy.get_param("~topics_config", package_config("topics.yaml")))
        self.data_cfg = load_yaml(rospy.get_param("~dataset_config", package_config("dataset.yaml")))
        model_source_dir = rospy.get_param(
            "~model_source_dir",
            os.path.abspath(os.path.join(
                os.path.dirname(__file__), "..", "model_artifacts"
            )),
        )
        checkpoint_path = rospy.get_param(
            "~checkpoint_path",
            "/home/acca/Downloads/best.pt",
        )
        if model_source_dir not in sys.path:
            sys.path.insert(0, model_source_dir)
        from multimodal_planner_v5.model import ModelConfig
        from multimodal_planner_v8.model import SpatialResidualPlannerV8
        from multimodal_planner_v8.velocity_planner import (
            VelocityPlannerConfig,
            build_mpc_path,
            plan_velocity_profile,
        )

        torch.serialization.add_safe_globals([PosixPath])
        checkpoint = torch.load(
            checkpoint_path,
            map_location="cpu",
            weights_only=True,
        )
        use_cuda = torch.cuda.is_available() and rospy.get_param("~use_cuda", True)
        self.device = torch.device("cuda" if use_cuda else "cpu")
        model_config = dict(checkpoint["model_config"])
        model_config["pretrained_camera"] = False
        self.model = SpatialResidualPlannerV8(
            ModelConfig(**model_config)
        ).to(self.device)
        self.model.load_state_dict(checkpoint["model_state"], strict=True)
        self.model.eval()
        self.path_spacing = float(rospy.get_param("~path_spacing", 0.1))
        if self.path_spacing <= 0.0:
            raise ValueError("path_spacing must be positive")
        self.path_length = float(rospy.get_param("~path_length", 80.0))
        self.velocity_lookahead = float(
            rospy.get_param("~velocity_lookahead", 1.0)
        )
        self.build_mpc_path = build_mpc_path
        self.plan_velocity_profile = plan_velocity_profile
        self.velocity_config = VelocityPlannerConfig(
            competition_max_speed_mps=float(
                rospy.get_param("~competition_max_speed_mps", 25.0)
            ),
            max_lateral_acceleration_mps2=float(
                rospy.get_param("~max_lateral_acceleration_mps2", 3.0)
            ),
            max_acceleration_mps2=float(
                rospy.get_param("~max_acceleration_mps2", 2.5)
            ),
            max_deceleration_mps2=float(
                rospy.get_param("~max_deceleration_mps2", 5.0)
            ),
            path_length_m=self.path_length,
            path_interval_m=self.path_spacing,
            stop_margin_m=float(rospy.get_param("~stop_margin_m", 2.0)),
        )
        self.use_amp = self.device.type == "cuda"

        self.history_size = checkpoint["model_config"]["history_frames"]
        self.history_interval = self.data_cfg["history_interval"]
        self.sample_period = self.data_cfg["sample_period"]
        self.last_encoded_stamp = float("-inf")
        self.dynamic_cache = deque(maxlen=32)
        self.static_lock = threading.Lock()
        self.map_input = self.route_input = self.route_msg = None
        self.static_revision = 0
        self.encoded_static_revision = -1
        self.encoded_static = None
        self.gps_lock = threading.Lock()
        self.latest_gps = NavSatFix()
        self.status_lock = threading.Lock()
        self.latest_vehicle_status = None
        self.lidar_lock = threading.Lock()
        self.latest_lidar = None
        self.lidar_sync_tolerance = float(
            rospy.get_param(
                "~lidar_sync_tolerance",
                self.data_cfg.get("sync_tolerance", 0.1),
            )
        )

        self.prediction_lock = threading.Lock()
        self.latest_output = None
        self.latest_prediction_time = self.latest_prediction_stamp = None
        self.prediction_timeout = float(rospy.get_param("~prediction_timeout", 0.3))
        publish_hz = float(rospy.get_param("~planning_publish_hz", 20.0))

        self.path_publisher = rospy.Publisher(self.topics["prediction_topic"], Path, queue_size=1)
        self.local_path_publisher = rospy.Publisher(
            "/multimodal_learning/predicted_path_local", Path, queue_size=1
        )
        self.speed_publisher = rospy.Publisher(self.topics["target_speed_topic"], Float32MultiArray, queue_size=1)
        self.mpc_speed_publisher = rospy.Publisher("/target_velocity", Float32, queue_size=1)
        self.kph_speed_publisher = rospy.Publisher("/target_velocity_kph", Float32, queue_size=1)
        self.mode_publisher = rospy.Publisher(self.topics["mode_score_topic"], Float32MultiArray, queue_size=1)
        rospy.Subscriber(self.topics["local_route_topic"], Path, self.route_callback, queue_size=1)
        rospy.Subscriber(self.topics["mgeo_token_topic"], Float32MultiArray, self.map_callback, queue_size=1)
        rospy.Subscriber(self.topics["gps_topic"], NavSatFix, self.gps_callback, queue_size=1)
        rospy.Subscriber(
            self.topics["vehicle_status_topic"],
            EgoVehicleStatus,
            self.status_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.topics["lidar_topic"],
            PointCloud2,
            self.lidar_callback,
            queue_size=1,
        )

        def camera_type(topic):
            return CompressedImage if topic.rstrip("/").endswith("/compressed") else Image

        subscribers = [
            message_filters.Subscriber(
                self.topics["camera_front_topic"], camera_type(self.topics["camera_front_topic"])
            ),
            message_filters.Subscriber(
                self.topics["camera_left_topic"], camera_type(self.topics["camera_left_topic"])
            ),
            message_filters.Subscriber(
                self.topics["camera_right_topic"], camera_type(self.topics["camera_right_topic"])
            ),
            message_filters.Subscriber(self.topics["odom_topic"], Odometry),
            message_filters.Subscriber(self.topics["imu_topic"], Imu),
        ]
        self.sync = message_filters.ApproximateTimeSynchronizer(
            subscribers,
            self.topics["queue_size"],
            float(rospy.get_param("~sync_slop", 0.25)),
        )
        self.sync.registerCallback(self.dynamic_callback)
        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / max(publish_hz, 1.0)), self.publish_prediction)
        rospy.loginfo(
            "Planner V8 best.pt epoch %d loaded on %s: %d-frame history, "
            "%.1fm path at %.2fm spacing",
            int(checkpoint.get("epoch", -1)) + 1,
            self.device,
            self.history_size,
            self.path_length,
            self.path_spacing,
        )

    def gps_callback(self, msg):
        with self.gps_lock:
            self.latest_gps = msg

    def status_callback(self, msg):
        with self.status_lock:
            self.latest_vehicle_status = msg

    def lidar_callback(self, msg):
        with self.lidar_lock:
            self.latest_lidar = msg

    def map_callback(self, msg):
        cfg = self.data_cfg
        try:
            value, _mask = mgeo_tokens_from_msg(
                msg, cfg["max_map_tokens"], cfg["map_token_dim"], True
            )
        except (ValueError, TypeError) as error:
            rospy.logwarn_throttle(2.0, "Invalid V8 MGeo tokens: %s", error)
            return
        with self.static_lock:
            self.map_input = value
            self.static_revision += 1
        rospy.loginfo_once("Planner V8 MGeo input ready")

    def route_callback(self, msg):
        cfg = self.data_cfg
        try:
            value, _mask = route_tokens_from_msg(
                msg, cfg["max_route_tokens"], 50.0,
                cfg.get("local_route_frame", "base_link"), False,
            )
        except (ValueError, TypeError) as error:
            rospy.logwarn_throttle(2.0, "Invalid V8 route: %s", error)
            return
        with self.static_lock:
            self.route_input, self.route_msg = value, msg
            self.static_revision += 1
        rospy.loginfo_once("Planner V8 local route input ready")

    def tensor(self, value):
        return torch.from_numpy(value).unsqueeze(0).to(self.device)

    def get_static_cache(self):
        with self.static_lock:
            if self.map_input is None or self.route_input is None:
                return None
            revision = self.static_revision
            map_tokens = self.map_input
            route_tokens = self.route_input
        if revision != self.encoded_static_revision:
            self.encoded_static = (
                self.tensor(map_tokens).float(),
                self.tensor(route_tokens).float(),
            )
            self.encoded_static_revision = revision
        return self.encoded_static

    def select_history(self, current_stamp):
        if len(self.dynamic_cache) < self.history_size:
            return None
        stamps = np.asarray([item[0] for item in self.dynamic_cache])
        selected = []
        for offset in reversed(range(self.history_size)):
            target = current_stamp - offset * self.history_interval
            index = int(np.argmin(np.abs(stamps - target)))
            if abs(stamps[index] - target) > 0.15:
                return None
            selected.append(self.dynamic_cache[index][1])
        return tuple(
            torch.stack([frame[field] for frame in selected], dim=1)
            for field in range(5)
        )

    @staticmethod
    def weighted_ego_vector(odom, imu, vehicle_status, gps, stamp, route):
        if vehicle_status is None:
            vx = float(odom.twist.twist.linear.x)
            vy = float(odom.twist.twist.linear.y)
            longitudinal_accel = float(imu.linear_acceleration.x)
            steering = 0.0
        else:
            velocity = vehicle_status.velocity
            vx = float(velocity.x) * 0.2777777777777778
            vy = float(velocity.y) * 0.2777777777777778
            longitudinal_accel = float(vehicle_status.acceleration.x)
            steering = math.radians(float(getattr(
                vehicle_status, "steer",
                getattr(vehicle_status, "wheel_angle", 0.0)
            )))
        speed = math.hypot(vx, vy)
        vehicle = np.asarray(
            [speed, vx, vy, steering, longitudinal_accel], np.float32
        ) / np.asarray([30.0, 30.0, 10.0, 0.7, 10.0], np.float32)
        imu_value = np.asarray([
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z,
        ], np.float32) / np.asarray([5.0, 5.0, 5.0, 20.0, 20.0, 20.0], np.float32)
        gps_stamp = getattr(getattr(gps, "header", None), "stamp", None)
        gps_time = gps_stamp.to_sec() if gps_stamp is not None else 0.0
        gps_age = max(0.0, stamp - gps_time) if gps_time > 0.0 else 1e6
        gps_valid = float(
            getattr(getattr(gps, "status", None), "status", -1) >= 0
            and np.isfinite(getattr(gps, "latitude", np.nan))
            and np.isfinite(getattr(gps, "longitude", np.nan))
        )
        covariance = np.asarray(odom.pose.covariance, np.float32)
        localization_confidence = math.exp(
            -math.sqrt(max(float(covariance[0] + covariance[7]), 0.0)) / 5.0
        )
        route_confidence = float(route is not None and len(route.poses) > 1)
        health = np.asarray([
            gps_valid, min(gps_age / 15.0, 1.0), localization_confidence,
            route_confidence, 0.0,
        ], np.float32)
        return np.concatenate((vehicle, imu_value, health))

    @staticmethod
    def current_speed_mps(odom, vehicle_status):
        if vehicle_status is None:
            velocity = odom.twist.twist.linear
            return math.hypot(float(velocity.x), float(velocity.y))
        velocity = vehicle_status.velocity
        return math.hypot(
            float(velocity.x), float(velocity.y)
        ) * 0.2777777777777778

    @staticmethod
    def route_xy_in_base_link(route, odom):
        if route is None or len(route.poses) < 2:
            raise ValueError("V8 requires a non-empty local route")
        points = np.asarray([
            [pose.pose.position.x, pose.pose.position.y]
            for pose in route.poses
        ], dtype=np.float32)
        frame = route.header.frame_id.lstrip("/")
        if frame in ("base_link", "base_footprint"):
            return points
        if frame != "map":
            raise ValueError(
                "unsupported local route frame: %s" % route.header.frame_id
            )
        orientation = odom.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        delta = points - np.asarray([
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
        ], dtype=np.float32)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        return np.column_stack((
            cos_yaw * delta[:, 0] + sin_yaw * delta[:, 1],
            -sin_yaw * delta[:, 0] + cos_yaw * delta[:, 1],
        )).astype(np.float32)

    def dynamic_callback(self, front_msg, left_msg, right_msg, odom_msg, imu_msg):
        rospy.loginfo_once("Planner V8 synchronized sensor callback active")
        stamp = odom_msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
        if stamp - self.last_encoded_stamp < self.sample_period - 1e-4:
            return
        cfg = self.data_cfg
        with self.gps_lock:
            gps_msg = self.latest_gps
        with self.status_lock:
            status_msg = self.latest_vehicle_status
        with self.lidar_lock:
            lidar_msg = self.latest_lidar
        with self.static_lock:
            route_msg = self.route_msg
        if lidar_msg is None:
            rospy.logerr_throttle(
                2.0,
                "Planner V8 waiting for required LiDAR topic %s; "
                "prediction is disabled (zero-BEV fallback removed)",
                self.topics["lidar_topic"],
            )
            return
        lidar_stamp = lidar_msg.header.stamp.to_sec()
        if lidar_stamp <= 0.0 or abs(stamp - lidar_stamp) > self.lidar_sync_tolerance:
            rospy.logerr_throttle(
                2.0,
                "Planner V8 LiDAR is stale/unsynchronized: age %.3fs "
                "(limit %.3fs); prediction is disabled",
                abs(stamp - lidar_stamp),
                self.lidar_sync_tolerance,
            )
            return
        try:
            front = image_from_msg(front_msg, cfg["front_image_width"], cfg["front_image_height"])
            left = image_from_msg(left_msg, cfg["side_image_width"], cfg["side_image_height"])
            right = image_from_msg(right_msg, cfg["side_image_width"], cfg["side_image_height"])
            lidar = points_to_bev(
                pointcloud2_to_xyz(lidar_msg),
                cfg["bev_x_range"],
                cfg["bev_y_range"],
                cfg["bev_height"],
                cfg["bev_width"],
            )
            ego = self.weighted_ego_vector(
                odom_msg, imu_msg, status_msg, gps_msg, stamp, route_msg
            )
            route_xy = self.route_xy_in_base_link(route_msg, odom_msg)
            current_speed = self.current_speed_mps(
                odom_msg, status_msg
            )
        except (ValueError, TypeError) as error:
            rospy.logwarn_throttle(2.0, "Invalid dynamic V8 input: %s", error)
            return

        with torch.no_grad(), torch.cuda.amp.autocast(enabled=self.use_amp):
            mean = torch.tensor(
                [0.485, 0.456, 0.406], device=self.device
            ).view(1, 3, 1, 1)
            std = torch.tensor(
                [0.229, 0.224, 0.225], device=self.device
            ).view(1, 3, 1, 1)
            dynamic = (
                (self.tensor(front).float() / 255.0 - mean) / std,
                (self.tensor(left).float() / 255.0 - mean) / std,
                (self.tensor(right).float() / 255.0 - mean) / std,
                self.tensor(lidar).float(),
                self.tensor(ego).float(),
            )
            self.dynamic_cache.append((stamp, dynamic))
            self.last_encoded_stamp = stamp
            history = self.select_history(stamp)
            static_cache = self.get_static_cache()
            if history is None or static_cache is None:
                return

            rospy.loginfo_once("Planner V8 history/static inputs ready")
            mgeo, route = static_cache
            output = self.model(*history, mgeo, route)
            rospy.loginfo_once("Planner V8 first forward completed")
            lateral_residual = (
                output["lateral_residual_m"][0].float().cpu().numpy()
            )
            residual_anchors = (
                output["spatial_stations_m"][0].float().cpu().numpy()
            )
            state_probabilities = (
                output["motion_state_probabilities"][0]
                .float().cpu().numpy()
            )
            drive = int(
                output["motion_state_prediction"][0].item()
            ) == 1
            try:
                planned_path = self.build_mpc_path(
                    route_xy,
                    lateral_residual,
                    residual_anchors,
                    path_length_m=self.path_length,
                    interval_m=self.path_spacing,
                )
                speeds = self.plan_velocity_profile(
                    planned_path,
                    current_speed,
                    self.velocity_config,
                    drive=drive,
                )
            except ValueError as error:
                rospy.logerr_throttle(
                    2.0,
                    "Planner V8 path/velocity planning failed: %s",
                    error,
                )
                return
            positions = planned_path["xy_m"]
            yaws = planned_path["yaw_rad"]
            local_trajectory = np.column_stack((
                positions, yaws, speeds
            ))
            lookahead_distance = max(
                current_speed * self.velocity_lookahead,
                self.path_spacing,
            )
            target_speed = float(np.interp(
                min(lookahead_distance, self.path_length),
                planned_path["station_m"],
                speeds,
            ))

            odom_orientation = odom_msg.pose.pose.orientation
            vehicle_yaw = math.atan2(
                2.0 * (
                    odom_orientation.w * odom_orientation.z
                    + odom_orientation.x * odom_orientation.y
                ),
                1.0 - 2.0 * (
                    odom_orientation.y * odom_orientation.y
                    + odom_orientation.z * odom_orientation.z
                ),
            )
            cos_yaw, sin_yaw = math.cos(vehicle_yaw), math.sin(vehicle_yaw)
            map_positions = np.column_stack((
                odom_msg.pose.pose.position.x
                + cos_yaw * positions[:, 0] - sin_yaw * positions[:, 1],
                odom_msg.pose.pose.position.y
                + sin_yaw * positions[:, 0] + cos_yaw * positions[:, 1],
            ))
            map_yaws = np.arctan2(
                np.sin(yaws + vehicle_yaw),
                np.cos(yaws + vehicle_yaw),
            )
            map_trajectory = np.column_stack((
                map_positions, map_yaws, speeds
            ))
            cpu_output = {
                "trajectory": map_trajectory,
                "local_trajectory": local_trajectory,
                "mode_scores": state_probabilities,
                "target_speed": target_speed,
                "drive": drive,
            }

        with self.prediction_lock:
            self.latest_output = cpu_output
            self.latest_prediction_time = rospy.Time.now()
            self.latest_prediction_stamp = odom_msg.header.stamp or self.latest_prediction_time

    def publish_prediction(self, _event):
        with self.prediction_lock:
            if self.latest_output is None:
                return
            trajectory = self.latest_output["trajectory"].copy()
            local_trajectory = self.latest_output["local_trajectory"].copy()
            mode_scores = self.latest_output["mode_scores"].copy()
            target_speed_mps = float(self.latest_output["target_speed"])
            drive = bool(self.latest_output["drive"])
            update_time, prediction_stamp = self.latest_prediction_time, self.latest_prediction_stamp
        if (rospy.Time.now() - update_time).to_sec() > self.prediction_timeout:
            rospy.logwarn_throttle(2.0, "Planner V8 prediction stale; publication paused")
            return

        path = Path()
        path.header.stamp = prediction_stamp
        path.header.frame_id = "map"
        for point in trajectory:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(point[0]), float(point[1])
            pose.pose.orientation.z = math.sin(float(point[2]) * 0.5)
            pose.pose.orientation.w = math.cos(float(point[2]) * 0.5)
            path.poses.append(pose)
        local_path = Path()
        local_path.header.stamp = prediction_stamp
        local_path.header.frame_id = "base_link"
        for point in local_trajectory:
            pose = PoseStamped()
            pose.header = local_path.header
            pose.pose.position.x, pose.pose.position.y = float(point[0]), float(point[1])
            pose.pose.orientation.z = math.sin(float(point[2]) * 0.5)
            pose.pose.orientation.w = math.cos(float(point[2]) * 0.5)
            local_path.poses.append(pose)
        speeds_kph = np.maximum(trajectory[:, 3], 0.0) * 3.6
        self.speed_publisher.publish(Float32MultiArray(data=speeds_kph.tolist()))
        if len(trajectory):
            predicted_speeds_mps = np.maximum(trajectory[:, 3], 0.0)
            target_speed_kph = target_speed_mps * 3.6
            self.mpc_speed_publisher.publish(Float32(data=target_speed_mps))
            self.kph_speed_publisher.publish(Float32(data=target_speed_kph))
            rospy.loginfo_throttle(
                1.0,
                "Planner V8 target speed: %.3f m/s (%.2f km/h, %s; "
                "profile %.3f-%.3f m/s)",
                target_speed_mps,
                target_speed_kph,
                "DRIVE" if drive else "STOP",
                float(np.min(predicted_speeds_mps)),
                float(np.max(predicted_speeds_mps)),
            )
        self.path_publisher.publish(path)
        self.local_path_publisher.publish(local_path)
        self.mode_publisher.publish(Float32MultiArray(data=mode_scores.tolist()))


if __name__ == "__main__":
    rospy.init_node("multimodal_inference")
    InferenceNode()
    rospy.spin()
