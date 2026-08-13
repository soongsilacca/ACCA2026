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
from geometry_msgs.msg import Point, PoseStamped
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CompressedImage, Image, Imu, NavSatFix, PointCloud2
from std_msgs.msg import Float32, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

from multimodal_learning.bag_tokens import route_tokens_from_msg
from multimodal_learning.io_utils import load_yaml, package_config
from multimodal_learning.lidar_bev import pointcloud2_to_xyz, points_to_bev
from multimodal_learning.sensor_preprocess import image_from_msg


class InferenceNode:
    def __init__(self):
        self.topics = load_yaml(rospy.get_param("~topics_config", package_config("topics.yaml")))
        self.data_cfg = load_yaml(rospy.get_param("~dataset_config", package_config("dataset.yaml")))
        model_source_dir = rospy.get_param(
            "~model_source_dir",
            "/home/acca",
        )
        checkpoint_path = rospy.get_param(
            "~checkpoint_path",
            "/home/acca/latest.pt",
        )
        if model_source_dir not in sys.path:
            sys.path.insert(0, model_source_dir)
        artifact_dir = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "model_artifacts"
        ))
        if artifact_dir not in sys.path:
            sys.path.insert(0, artifact_dir)
        from multimodal_planner_v9.data import (
            ACTION_AVOID,
            ACTION_DRIVE,
            ACTION_NAMES,
            ACTION_STOP,
        )
        from multimodal_planner_v9.model import (
            ModelConfig,
            SpatialResidualSpeedPlannerV9,
        )
        from multimodal_planner_v8.velocity_planner import (
            build_mpc_path_from_candidate,
            plan_curvature_speed_profile,
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
        self.model = SpatialResidualSpeedPlannerV9(
            ModelConfig(**model_config)
        ).to(self.device)
        self.model.load_state_dict(checkpoint["model_state"], strict=True)
        self.model.eval()
        self.action_drive = ACTION_DRIVE
        self.action_stop = ACTION_STOP
        self.action_avoid = ACTION_AVOID
        self.action_names = ACTION_NAMES
        self.action_queue_size = max(
            int(rospy.get_param("~action_queue_size", 5)), 1
        )
        self.action_switch_count = max(
            int(rospy.get_param("~action_switch_count", 3)), 1
        )
        if self.action_switch_count > self.action_queue_size:
            raise ValueError(
                "action_switch_count cannot exceed action_queue_size"
            )
        self.immediate_stop_probability = float(
            rospy.get_param("~immediate_stop_probability", 0.8)
        )
        if not 0.0 <= self.immediate_stop_probability <= 1.0:
            raise ValueError("immediate_stop_probability must be in [0,1]")
        self.immediate_avoid_probability = float(
            rospy.get_param("~immediate_avoid_probability", 0.3)
        )
        if not 0.0 <= self.immediate_avoid_probability <= 1.0:
            raise ValueError("immediate_avoid_probability must be in [0,1]")
        self.avoid_min_hold_sec = float(
            rospy.get_param("~avoid_min_hold_sec", 4.0)
        )
        if self.avoid_min_hold_sec < 0.0:
            raise ValueError("avoid_min_hold_sec cannot be negative")
        self.enable_stop_action = bool(
            rospy.get_param("~enable_stop_action", False)
        )
        self.delta_only_mode = bool(
            rospy.get_param("~delta_only_mode", True)
        )
        self.delta_min_speed_kph = float(
            rospy.get_param("~delta_min_speed_kph", 30.0)
        )
        if self.delta_min_speed_kph < 0.0:
            raise ValueError("delta_min_speed_kph cannot be negative")
        self.action_history = deque(maxlen=self.action_queue_size)
        self.runtime_action = self.action_drive
        self.avoid_hold_until = rospy.Time(0)
        self.capture_avoid_path = False
        self.latched_avoid_path = None
        self.path_spacing = float(rospy.get_param("~path_spacing", 0.1))
        if self.path_spacing <= 0.0:
            raise ValueError("path_spacing must be positive")
        self.path_length = float(rospy.get_param("~path_length", 80.0))
        self.mpc_route_start_index = int(
            rospy.get_param("~mpc_route_start_index", 2)
        )
        if self.mpc_route_start_index < 0:
            raise ValueError("mpc_route_start_index cannot be negative")
        self.lateral_residual_deadband_m = float(
            rospy.get_param("~lateral_residual_deadband_m", 0.1)
        )
        if self.lateral_residual_deadband_m < 0.0:
            raise ValueError("lateral_residual_deadband_m cannot be negative")
        self.build_mpc_path = build_mpc_path_from_candidate
        self.plan_curvature_speed_profile = plan_curvature_speed_profile
        self.max_lateral_acceleration_mps2 = float(
            rospy.get_param("~max_lateral_acceleration_mps2", 3.0)
        )
        self.max_curve_deceleration_mps2 = float(
            rospy.get_param("~max_curve_deceleration_mps2", 2.0)
        )
        self.curvature_smoothing_m = float(
            rospy.get_param("~curvature_smoothing_m", 1.0)
        )
        if self.max_lateral_acceleration_mps2 <= 0.0:
            raise ValueError("max_lateral_acceleration_mps2 must be positive")
        if self.max_curve_deceleration_mps2 <= 0.0:
            raise ValueError("max_curve_deceleration_mps2 must be positive")
        if self.curvature_smoothing_m < 0.0:
            raise ValueError("curvature_smoothing_m cannot be negative")
        self.use_amp = self.device.type == "cuda"

        self.history_size = checkpoint["model_config"]["history_frames"]
        self.history_interval = self.data_cfg["history_interval"]
        self.sample_period = self.data_cfg["sample_period"]
        self.model_inference_period = float(
            rospy.get_param("~model_inference_period", 0.3)
        )
        if self.model_inference_period <= 0.0:
            raise ValueError("model_inference_period must be positive")
        self.last_encoded_stamp = float("-inf")
        self.last_inference_stamp = float("-inf")
        self.dynamic_cache = deque(maxlen=32)
        self.static_lock = threading.Lock()
        self.route_input = self.route_msg = None
        self.static_revision = 0
        self.encoded_static_revision = -1
        self.encoded_static = None
        self.gps_lock = threading.Lock()
        self.latest_gps = NavSatFix()
        self.status_lock = threading.Lock()
        self.latest_vehicle_status = None
        self.mgeo_speed_lock = threading.Lock()
        self.latest_mgeo_target_speed = None
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
        self.path_marker_publisher = rospy.Publisher(
            rospy.get_param(
                "~path_marker_topic",
                "/multimodal_learning/predicted_path_markers",
            ),
            MarkerArray,
            queue_size=1,
        )
        self.local_path_publisher = rospy.Publisher(
            "/multimodal_learning/predicted_path_local", Path, queue_size=1
        )
        self.speed_publisher = rospy.Publisher(self.topics["target_speed_topic"], Float32MultiArray, queue_size=1)
        self.mpc_speed_publisher = rospy.Publisher("/target_velocity", Float32, queue_size=1)
        self.kph_speed_publisher = rospy.Publisher("/target_velocity_kph", Float32, queue_size=1)
        self.mode_publisher = rospy.Publisher(self.topics["mode_score_topic"], Float32MultiArray, queue_size=1)
        self.action_score_publisher = rospy.Publisher(
            "/multimodal_learning/action_scores",
            Float32MultiArray,
            queue_size=1,
        )
        self.runtime_action_publisher = rospy.Publisher(
            "/multimodal_learning/runtime_action",
            String,
            queue_size=1,
        )
        self.last_announced_runtime_action = None
        rospy.Subscriber(self.topics["local_route_topic"], Path, self.route_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param(
                "~mgeo_target_velocity_topic",
                "/mgeo_target_velocity",
            ),
            Float32,
            self.mgeo_target_speed_callback,
            queue_size=1,
        )
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
            "Planner V9 %s epoch %d loaded on %s: %d-frame history, "
            "%.1fm path at %.2fm spacing, local route start index %d, "
            "lateral residual deadband %.2fm, action queue %d/%d, "
            "AVOID immediate >= %.2f with %.1fs minimum hold, STOP %s, "
            "delta-only %s (minimum %.1f km/h), model period %.2fs (%.2fHz)",
            os.path.basename(checkpoint_path),
            int(checkpoint.get("epoch", -1)) + 1,
            self.device,
            self.history_size,
            self.path_length,
            self.path_spacing,
            self.mpc_route_start_index,
            self.lateral_residual_deadband_m,
            self.action_switch_count,
            self.action_queue_size,
            self.immediate_avoid_probability,
            self.avoid_min_hold_sec,
            "enabled" if self.enable_stop_action else "ignored",
            "enabled" if self.delta_only_mode else "disabled",
            self.delta_min_speed_kph,
            self.model_inference_period,
            1.0 / self.model_inference_period,
        )

    def gps_callback(self, msg):
        with self.gps_lock:
            self.latest_gps = msg

    def status_callback(self, msg):
        with self.status_lock:
            self.latest_vehicle_status = msg

    def mgeo_target_speed_callback(self, msg):
        with self.mgeo_speed_lock:
            self.latest_mgeo_target_speed = max(float(msg.data), 0.0)
        rospy.loginfo_once("Planner V9 MGeo target speed input ready")

    def lidar_callback(self, msg):
        with self.lidar_lock:
            self.latest_lidar = msg

    def route_callback(self, msg):
        cfg = self.data_cfg
        try:
            value, _mask = route_tokens_from_msg(
                msg, cfg["max_route_tokens"], 50.0,
                cfg.get("local_route_frame", "base_link"), False,
            )
        except (ValueError, TypeError) as error:
            rospy.logwarn_throttle(2.0, "Invalid V9 route: %s", error)
            return
        with self.static_lock:
            self.route_input, self.route_msg = value, msg
            self.static_revision += 1
        rospy.loginfo_once("Planner V9 local route input ready")

    def tensor(self, value):
        return torch.from_numpy(value).unsqueeze(0).to(self.device)

    def get_static_cache(self):
        with self.static_lock:
            if self.route_input is None:
                return None
            revision = self.static_revision
            route_tokens = self.route_input
        if revision != self.encoded_static_revision:
            self.encoded_static = self.tensor(route_tokens).float()
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
    def route_xy_in_base_link(route, odom):
        if route is None or len(route.poses) < 2:
            raise ValueError("V9 requires a non-empty local route")
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

    def select_runtime_action(self, probabilities):
        probabilities = np.asarray(probabilities, dtype=np.float32)
        if probabilities.shape != (3,) or not np.isfinite(probabilities).all():
            raise ValueError("V9 action probabilities must be finite [3]")

        now = rospy.Time.now()
        if (
            not self.enable_stop_action
            and self.runtime_action == self.action_avoid
            and now < self.avoid_hold_until
        ):
            # Do not collect DRIVE release votes during the minimum hold.
            return self.runtime_action

        if (
            not self.enable_stop_action
            and probabilities[self.action_avoid]
            >= self.immediate_avoid_probability
        ):
            # Enter AVOID on the current inference result. Resetting and
            # seeding the queue prevents preceding DRIVE votes from releasing
            # AVOID again on the very next frame.
            self.runtime_action = self.action_avoid
            self.action_history.clear()
            self.action_history.append(self.action_avoid)
            self.avoid_hold_until = now + rospy.Duration(
                self.avoid_min_hold_sec
            )
            self.capture_avoid_path = True
            return self.runtime_action

        if not self.enable_stop_action and self.runtime_action == self.action_avoid:
            # AVOID release alone is debounced: require DRIVE to win the
            # configured N/M queue (3/5 by default).
            predicted = (
                self.action_avoid
                if probabilities[self.action_avoid]
                > probabilities[self.action_drive]
                else self.action_drive
            )
            self.action_history.append(predicted)
            drive_votes = sum(
                action == self.action_drive for action in self.action_history
            )
            if drive_votes >= self.action_switch_count:
                self.runtime_action = self.action_drive
                self.latched_avoid_path = None
            return self.runtime_action

        if self.enable_stop_action:
            predicted = int(np.argmax(probabilities))
        else:
            # With STOP ignored, AVOID entry is handled only by the immediate
            # probability threshold above. Stay in DRIVE below that threshold.
            self.runtime_action = self.action_drive
            self.action_history.append(self.action_drive)
            return self.runtime_action
        self.action_history.append(predicted)

        # A confident STOP is applied immediately. Other transitions require
        # a short majority queue so a single camera frame cannot move the path.
        if (
            self.enable_stop_action
            and probabilities[self.action_stop]
            >= self.immediate_stop_probability
        ):
            self.runtime_action = self.action_stop
            return self.runtime_action

        counts = np.bincount(
            np.asarray(self.action_history, dtype=np.int64),
            minlength=3,
        )
        candidate = int(np.argmax(counts))
        if counts[candidate] >= self.action_switch_count:
            self.runtime_action = candidate
        return self.runtime_action

    def plan_avoid_speed_profile(
        self,
        path,
        anchors_m,
        candidate_speed_mps,
        external_speed_mps,
    ):
        """Blend V9 near-field delta-v with MGeo and curvature limits."""
        station = np.asarray(path["station_m"], dtype=np.float64)
        anchors = np.asarray(anchors_m, dtype=np.float64).reshape(-1)
        candidate = np.asarray(
            candidate_speed_mps, dtype=np.float64
        ).reshape(-1)
        if candidate.shape != anchors.shape:
            raise ValueError("V9 speed anchors and values must match")

        base_speed = max(float(external_speed_mps), 0.0)
        learned = np.interp(
            np.minimum(station, anchors[-1]),
            anchors,
            np.minimum(candidate, base_speed),
        )
        tail = station > anchors[-1]
        if tail.any():
            tail_span = max(self.path_length - anchors[-1], self.path_spacing)
            u = np.clip(
                (station[tail] - anchors[-1]) / tail_span,
                0.0,
                1.0,
            )
            smooth = 6.0 * u**5 - 15.0 * u**4 + 10.0 * u**3
            learned[tail] = (
                learned[tail] * (1.0 - smooth) + base_speed * smooth
            )

        curve_speed = self.plan_curvature_speed_profile(
            path,
            base_speed,
            max_lateral_acceleration_mps2=(
                self.max_lateral_acceleration_mps2
            ),
            max_deceleration_mps2=self.max_curve_deceleration_mps2,
            curvature_smoothing_m=self.curvature_smoothing_m,
        ).astype(np.float64)
        speed = np.minimum(learned, curve_speed)

        # Propagate a learned future reduction backwards so MPC receives a
        # feasible preceding-deceleration profile rather than a speed step.
        ds = np.diff(station)
        for index in range(len(speed) - 2, -1, -1):
            reachable = math.sqrt(
                max(
                    speed[index + 1] ** 2
                    + 2.0 * self.max_curve_deceleration_mps2 * ds[index],
                    0.0,
                )
            )
            speed[index] = min(speed[index], reachable)
        return speed.astype(np.float32), curve_speed.astype(np.float32)

    def plan_delta_only_speed_profile(
        self,
        path,
        sample_progress_m,
        candidate_speed_mps,
        external_speed_mps,
    ):
        """Use learned delta-v only, with a nonzero-road-speed floor."""
        station = np.asarray(path["station_m"], dtype=np.float64)
        progress = np.asarray(sample_progress_m, dtype=np.float64).reshape(-1)
        candidate = np.asarray(
            candidate_speed_mps, dtype=np.float64
        ).reshape(-1)
        if candidate.shape != progress.shape or len(progress) < 2:
            raise ValueError("V9 delta-v samples and path progress must match")

        base_speed = max(float(external_speed_mps), 0.0)
        if base_speed <= 0.0:
            return np.zeros(len(station), dtype=np.float32)
        minimum_speed = min(self.delta_min_speed_kph / 3.6, base_speed)
        learned_samples = np.clip(candidate, minimum_speed, base_speed)
        learned = np.interp(
            np.minimum(station, progress[-1]),
            progress,
            learned_samples,
        )
        tail = station > progress[-1]
        if tail.any():
            tail_span = max(self.path_length - progress[-1], self.path_spacing)
            u = np.clip(
                (station[tail] - progress[-1]) / tail_span,
                0.0,
                1.0,
            )
            smooth = 6.0 * u**5 - 15.0 * u**4 + 10.0 * u**3
            learned[tail] = (
                learned[tail] * (1.0 - smooth) + base_speed * smooth
            )
        return np.clip(learned, minimum_speed, base_speed).astype(np.float32)

    def dynamic_callback(self, front_msg, left_msg, right_msg, odom_msg, imu_msg):
        rospy.loginfo_once("Planner V9 synchronized sensor callback active")
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
                "Planner V9 waiting for required LiDAR topic %s; "
                "prediction is disabled (zero-BEV fallback removed)",
                self.topics["lidar_topic"],
            )
            return
        lidar_stamp = lidar_msg.header.stamp.to_sec()
        if lidar_stamp <= 0.0 or abs(stamp - lidar_stamp) > self.lidar_sync_tolerance:
            rospy.logerr_throttle(
                2.0,
                "Planner V9 LiDAR is stale/unsynchronized: age %.3fs "
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
        except (ValueError, TypeError) as error:
            rospy.logwarn_throttle(2.0, "Invalid dynamic V9 input: %s", error)
            return

        with torch.no_grad(), torch.amp.autocast(
            self.device.type, enabled=self.use_amp
        ):
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
            if (
                stamp - self.last_inference_stamp
                < self.model_inference_period - 1.0e-4
            ):
                return
            self.last_inference_stamp = stamp
            history = self.select_history(stamp)
            static_cache = self.get_static_cache()
            if history is None or static_cache is None:
                return

            with self.mgeo_speed_lock:
                mgeo_target_speed = self.latest_mgeo_target_speed
            if mgeo_target_speed is None:
                rospy.logwarn_throttle(
                    2.0,
                    "Planner V9 waiting for /mgeo_target_velocity; "
                    "safe target speed 0 m/s",
                )
                mgeo_target_speed = 0.0

            rospy.loginfo_once("Planner V9 history/route inputs ready")
            route = static_cache
            # The external speed profile is not a learned context input. V9
            # only uses it to scale its non-positive AVOID delta-v output.
            model_base_speed = max(float(mgeo_target_speed), 0.1)
            base_speed_profile = self.tensor(
                np.full(64, model_base_speed, dtype=np.float32)
            ).float()
            output = self.model(
                *history,
                base_speed_profile,
                route,
            )
            rospy.loginfo_once("Planner V9 first forward completed")
            base_candidate_path = (
                output["base_spatial_path_xy_m"][0]
                .float().cpu().numpy()
            )
            candidate_path = (
                output["candidate_spatial_path_xy_m"][0]
                .float().cpu().numpy()
            )
            action_probabilities = (
                output["action_probabilities"][0]
                .float().cpu().numpy()
            )
            runtime_action = self.select_runtime_action(
                action_probabilities
            )
            candidate_speed = (
                output["candidate_spatial_speed_mps"][0]
                .float().cpu().numpy()
            )
            use_candidate_path = (
                self.delta_only_mode
                or runtime_action == self.action_avoid
            )
            try:
                planned_path = self.build_mpc_path(
                    route_xy,
                    (
                        base_candidate_path
                        if use_candidate_path
                        else None
                    ),
                    (
                        candidate_path
                        if use_candidate_path
                        else None
                    ),
                    path_length_m=self.path_length,
                    interval_m=self.path_spacing,
                    route_start_index=self.mpc_route_start_index,
                    lateral_deadband_m=self.lateral_residual_deadband_m,
                )
            except ValueError as error:
                rospy.logerr_throttle(
                    2.0,
                    "Planner V9 path building failed: %s",
                    error,
                )
                return
            positions = planned_path["xy_m"]
            yaws = planned_path["yaw_rad"]
            if self.delta_only_mode:
                keep_indices = planned_path["candidate_keep_indices"]
                speeds = self.plan_delta_only_speed_profile(
                    planned_path,
                    planned_path["candidate_station_m"],
                    candidate_speed[keep_indices],
                    float(mgeo_target_speed),
                )
                curve_speeds = speeds
            elif runtime_action == self.action_stop:
                speeds = np.zeros(len(positions), dtype=np.float32)
                curve_speeds = speeds
            elif runtime_action == self.action_avoid:
                keep_indices = planned_path["candidate_keep_indices"]
                speeds, curve_speeds = self.plan_avoid_speed_profile(
                    planned_path,
                    planned_path["candidate_station_m"],
                    candidate_speed[keep_indices],
                    float(mgeo_target_speed),
                )
            else:
                speeds = self.plan_curvature_speed_profile(
                    planned_path,
                    float(mgeo_target_speed),
                    max_lateral_acceleration_mps2=(
                        self.max_lateral_acceleration_mps2
                    ),
                    max_deceleration_mps2=(
                        self.max_curve_deceleration_mps2
                    ),
                    curvature_smoothing_m=self.curvature_smoothing_m,
                )
                curve_speeds = speeds
            target_speed = float(speeds[0])
            curve_target_speed = float(curve_speeds[0])
            local_trajectory = np.column_stack((
                positions, yaws, speeds
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
            # The RViz base line must begin at the same real model point as
            # the delta-applied path; it must not reintroduce an s=0 prefix.
            base_positions = planned_path["base_xy_m"]
            base_map_positions = np.column_stack((
                odom_msg.pose.pose.position.x
                + cos_yaw * base_positions[:, 0]
                - sin_yaw * base_positions[:, 1],
                odom_msg.pose.pose.position.y
                + sin_yaw * base_positions[:, 0]
                + cos_yaw * base_positions[:, 1],
            ))
            delta_base_map = np.column_stack((
                odom_msg.pose.pose.position.x
                + cos_yaw * base_candidate_path[:, 0]
                - sin_yaw * base_candidate_path[:, 1],
                odom_msg.pose.pose.position.y
                + sin_yaw * base_candidate_path[:, 0]
                + cos_yaw * base_candidate_path[:, 1],
            ))
            delta_candidate_map = np.column_stack((
                odom_msg.pose.pose.position.x
                + cos_yaw * candidate_path[:, 0]
                - sin_yaw * candidate_path[:, 1],
                odom_msg.pose.pose.position.y
                + sin_yaw * candidate_path[:, 0]
                + cos_yaw * candidate_path[:, 1],
            ))
            prediction_stamp = (
                odom_msg.header.stamp
                if odom_msg.header.stamp != rospy.Time(0)
                else rospy.Time.now()
            )

            if not self.delta_only_mode and runtime_action == self.action_avoid:
                if self.capture_avoid_path or self.latched_avoid_path is None:
                    # Latch the completed Local Route + delta-d path in map
                    # coordinates. It must stay fixed in the world instead of
                    # being reattached to the moving ego origin every frame.
                    self.latched_avoid_path = {
                        "trajectory": map_trajectory.copy(),
                        "target_speed": target_speed,
                        "mgeo_target_speed": float(mgeo_target_speed),
                        "curve_target_speed": curve_target_speed,
                        "prediction_stamp": prediction_stamp,
                    }
                    self.capture_avoid_path = False
                else:
                    latched = self.latched_avoid_path
                    map_trajectory = latched["trajectory"].copy()
                    target_speed = float(latched["target_speed"])
                    mgeo_target_speed = float(latched["mgeo_target_speed"])
                    curve_target_speed = float(latched["curve_target_speed"])
                    prediction_stamp = latched["prediction_stamp"]

                    # Only the diagnostic local-frame copy changes with ego
                    # pose. The MPC-facing map path above remains identical.
                    delta = map_trajectory[:, :2] - np.asarray([
                        odom_msg.pose.pose.position.x,
                        odom_msg.pose.pose.position.y,
                    ], dtype=np.float64)
                    local_positions = np.column_stack((
                        cos_yaw * delta[:, 0] + sin_yaw * delta[:, 1],
                        -sin_yaw * delta[:, 0] + cos_yaw * delta[:, 1],
                    ))
                    local_yaws = np.arctan2(
                        np.sin(map_trajectory[:, 2] - vehicle_yaw),
                        np.cos(map_trajectory[:, 2] - vehicle_yaw),
                    )
                    local_trajectory = np.column_stack((
                        local_positions,
                        local_yaws,
                        map_trajectory[:, 3],
                    ))
            else:
                self.latched_avoid_path = None

            cpu_output = {
                "trajectory": map_trajectory,
                "local_trajectory": local_trajectory,
                "base_map_positions": base_map_positions,
                "delta_base_map": delta_base_map,
                "delta_candidate_map": delta_candidate_map,
                "action_scores": action_probabilities,
                "target_speed": target_speed,
                "mgeo_target_speed": float(mgeo_target_speed),
                "curve_target_speed": curve_target_speed,
                "runtime_action": runtime_action,
                "planning_mode": (
                    "DELTA_ONLY"
                    if self.delta_only_mode
                    else self.action_names[runtime_action]
                ),
                "prediction_stamp": prediction_stamp,
            }

        with self.prediction_lock:
            self.latest_output = cpu_output
            self.latest_prediction_time = rospy.Time.now()
            self.latest_prediction_stamp = cpu_output["prediction_stamp"]

    def publish_path_markers(
        self,
        trajectory,
        base_map_positions,
        delta_base_map,
        delta_candidate_map,
        action_scores,
        target_speed_mps,
        stamp,
        runtime_action,
        planning_mode,
    ):
        color_by_action = {
            self.action_drive: (0.10, 0.95, 0.20),
            self.action_stop: (0.95, 0.08, 0.08),
            self.action_avoid: (1.00, 0.55, 0.05),
        }
        red, green, blue = color_by_action.get(
            runtime_action, (1.0, 1.0, 1.0)
        )
        if planning_mode == "DELTA_ONLY":
            red, green, blue = (0.05, 0.75, 1.00)
        marker_array = MarkerArray()

        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = "map"
        line.ns = "v9_predicted_path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.16
        line.color.r = red
        line.color.g = green
        line.color.b = blue
        line.color.a = 0.95
        line.lifetime = rospy.Duration(0.25)

        points = Marker()
        points.header = line.header
        points.ns = line.ns
        points.id = 1
        points.type = Marker.SPHERE_LIST
        points.action = Marker.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = 0.11
        points.scale.y = 0.11
        points.scale.z = 0.11
        points.color.r = red
        points.color.g = green
        points.color.b = blue
        points.color.a = 0.55
        points.lifetime = line.lifetime

        for row in trajectory:
            point = Point()
            point.x = float(row[0])
            point.y = float(row[1])
            point.z = 0.15
            line.points.append(point)
            points.points.append(point)

        base_line = Marker()
        base_line.header = line.header
        base_line.ns = line.ns
        base_line.id = 3
        base_line.type = Marker.LINE_STRIP
        base_line.action = Marker.ADD
        base_line.pose.orientation.w = 1.0
        base_line.scale.x = 0.09
        base_line.color.r = 0.70
        base_line.color.g = 0.70
        base_line.color.b = 0.70
        base_line.color.a = 0.85
        base_line.lifetime = line.lifetime
        for row in base_map_positions:
            point = Point()
            point.x = float(row[0])
            point.y = float(row[1])
            point.z = 0.10
            base_line.points.append(point)

        delta_lines = Marker()
        delta_lines.header = line.header
        delta_lines.ns = line.ns
        delta_lines.id = 4
        delta_lines.type = Marker.LINE_LIST
        delta_lines.action = Marker.ADD
        delta_lines.pose.orientation.w = 1.0
        delta_lines.scale.x = 0.06
        delta_lines.color.r = 1.00
        delta_lines.color.g = 0.05
        delta_lines.color.b = 0.85
        delta_lines.color.a = 0.95
        delta_lines.lifetime = line.lifetime
        for base_row, candidate_row in zip(
            delta_base_map, delta_candidate_map
        ):
            base_point = Point()
            base_point.x = float(base_row[0])
            base_point.y = float(base_row[1])
            base_point.z = 0.35
            candidate_point = Point()
            candidate_point.x = float(candidate_row[0])
            candidate_point.y = float(candidate_row[1])
            candidate_point.z = 0.35
            delta_lines.points.extend((base_point, candidate_point))

        action_label = Marker()
        action_label.header = line.header
        action_label.ns = line.ns
        action_label.id = 2
        action_label.type = Marker.TEXT_VIEW_FACING
        action_label.action = Marker.ADD
        action_label.pose.orientation.w = 1.0
        if len(trajectory):
            action_label.pose.position.x = float(trajectory[0][0])
            action_label.pose.position.y = float(trajectory[0][1])
        action_label.pose.position.z = 1.5
        action_label.scale.z = 0.8
        action_label.color.r = red
        action_label.color.g = green
        action_label.color.b = blue
        action_label.color.a = 1.0
        action_label.text = (
            "%s | target %.1f km/h\n"
            "P D/S/A %.2f / %.2f / %.2f"
            % (
                planning_mode,
                target_speed_mps * 3.6,
                float(action_scores[self.action_drive]),
                float(action_scores[self.action_stop]),
                float(action_scores[self.action_avoid]),
            )
        )
        action_label.lifetime = line.lifetime

        marker_array.markers.extend((
            base_line,
            line,
            points,
            delta_lines,
            action_label,
        ))
        self.path_marker_publisher.publish(marker_array)

    def publish_prediction(self, _event):
        with self.prediction_lock:
            if self.latest_output is None:
                return
            trajectory = self.latest_output["trajectory"].copy()
            local_trajectory = self.latest_output["local_trajectory"].copy()
            base_map_positions = self.latest_output["base_map_positions"].copy()
            delta_base_map = self.latest_output["delta_base_map"].copy()
            delta_candidate_map = self.latest_output[
                "delta_candidate_map"
            ].copy()
            action_scores = self.latest_output["action_scores"].copy()
            target_speed_mps = float(self.latest_output["target_speed"])
            mgeo_target_speed_mps = float(
                self.latest_output["mgeo_target_speed"]
            )
            curve_target_speed_mps = float(
                self.latest_output["curve_target_speed"]
            )
            runtime_action = int(self.latest_output["runtime_action"])
            planning_mode = str(self.latest_output["planning_mode"])
            update_time, prediction_stamp = self.latest_prediction_time, self.latest_prediction_stamp
        if (rospy.Time.now() - update_time).to_sec() > self.prediction_timeout:
            rospy.logwarn_throttle(2.0, "Planner V9 prediction stale; publication paused")
            return

        action_name = planning_mode
        self.runtime_action_publisher.publish(String(data=action_name))
        if action_name != self.last_announced_runtime_action:
            previous_name = (
                "NONE"
                if self.last_announced_runtime_action is None
                else self.last_announced_runtime_action
            )
            rospy.logwarn(
                "Planner V9 runtime action changed: %s -> %s "
                "(P_DRIVE=%.3f P_STOP=%.3f P_AVOID=%.3f)",
                previous_name,
                action_name,
                float(action_scores[self.action_drive]),
                float(action_scores[self.action_stop]),
                float(action_scores[self.action_avoid]),
            )
            self.last_announced_runtime_action = action_name

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
            target_speed_kph = target_speed_mps * 3.6
            self.mpc_speed_publisher.publish(Float32(data=target_speed_mps))
            self.kph_speed_publisher.publish(Float32(data=target_speed_kph))
            rospy.loginfo_throttle(
                1.0,
                "Planner V9 target speed: %.3f m/s (%.2f km/h, %s; "
                "MGeo %.3f m/s, applied profile %.3f m/s; "
                "P=%.3f/%.3f/%.3f)",
                target_speed_mps,
                target_speed_kph,
                planning_mode,
                mgeo_target_speed_mps,
                curve_target_speed_mps,
                float(action_scores[self.action_drive]),
                float(action_scores[self.action_stop]),
                float(action_scores[self.action_avoid]),
            )
        self.path_publisher.publish(path)
        self.local_path_publisher.publish(local_path)
        self.publish_path_markers(
            trajectory,
            base_map_positions,
            delta_base_map,
            delta_candidate_map,
            action_scores,
            target_speed_mps,
            prediction_stamp,
            runtime_action,
            planning_mode,
        )
        # Existing MPC contract is [STOP, proceed]. When STOP routing is
        # disabled, publish an unconditional proceed score so the downstream
        # brake latch cannot be engaged by the ignored raw STOP probability.
        if self.enable_stop_action:
            mpc_mode_scores = [
                float(action_scores[self.action_stop]),
                float(
                    action_scores[self.action_drive]
                    + action_scores[self.action_avoid]
                ),
            ]
        else:
            mpc_mode_scores = [0.0, 1.0]
        self.mode_publisher.publish(
            Float32MultiArray(data=mpc_mode_scores)
        )
        self.action_score_publisher.publish(
            Float32MultiArray(data=action_scores.tolist())
        )


if __name__ == "__main__":
    rospy.init_node("multimodal_inference")
    InferenceNode()
    rospy.spin()
