#!/usr/bin/env python3
from collections import deque
import io
import math
from pathlib import PosixPath
import sys
import threading
import zipfile

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
        model_zip = rospy.get_param(
            "~model_source_zip",
            "/media/acca/UBUNTU 20_0/multimodal_planner_v2.zip",
        )
        artifact_zip = rospy.get_param(
            "~artifact_zip",
            "/media/acca/UBUNTU 20_0/trajectory_smoothing.zip",
        )
        if model_zip not in sys.path:
            sys.path.insert(0, model_zip)
        if artifact_zip not in sys.path:
            sys.path.insert(0, artifact_zip)
        from multimodal_planner_v2.model import (
            ModelConfig,
            MultiViewTemporalTrajectoryPlannerV2,
        )
        from trajectory_smoothing import (
            interpolate_positions,
            path_yaws,
            smooth_positions,
        )

        torch.serialization.add_safe_globals([PosixPath])
        with zipfile.ZipFile(artifact_zip) as archive:
            checkpoint = torch.load(
                io.BytesIO(archive.read("best.pt")),
                map_location="cpu",
                weights_only=True,
            )
        use_cuda = torch.cuda.is_available() and rospy.get_param("~use_cuda", True)
        self.device = torch.device("cuda" if use_cuda else "cpu")
        model_config = dict(checkpoint["model_config"])
        model_config["pretrained_camera"] = False
        self.model = MultiViewTemporalTrajectoryPlannerV2(
            ModelConfig(**model_config)
        ).to(self.device)
        self.model.load_state_dict(checkpoint["model_state"], strict=True)
        self.model.eval()
        self.smooth_positions = smooth_positions
        self.interpolate_positions = interpolate_positions
        self.path_yaws = path_yaws
        self.smoothing_strength = float(rospy.get_param("~smoothing_strength", 8.0))
        self.points_per_segment = int(rospy.get_param("~points_per_segment", 4))
        self.target_speed_lookahead = float(
            rospy.get_param("~target_speed_lookahead", 1.0)
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

        self.prediction_lock = threading.Lock()
        self.latest_output = None
        self.latest_prediction_time = self.latest_prediction_stamp = None
        self.prediction_timeout = float(rospy.get_param("~prediction_timeout", 0.3))
        publish_hz = float(rospy.get_param("~planning_publish_hz", 20.0))

        self.path_publisher = rospy.Publisher(self.topics["prediction_topic"], Path, queue_size=1)
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
            message_filters.Subscriber(self.topics["lidar_topic"], PointCloud2),
            message_filters.Subscriber(self.topics["odom_topic"], Odometry),
            message_filters.Subscriber(self.topics["imu_topic"], Imu),
        ]
        self.sync = message_filters.ApproximateTimeSynchronizer(
            subscribers, self.topics["queue_size"], self.topics["sync_slop"]
        )
        self.sync.registerCallback(self.dynamic_callback)
        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / max(publish_hz, 1.0)), self.publish_prediction)
        rospy.loginfo(
            "Planner V2 weighted checkpoint loaded on %s: %d-frame smoothed trajectory enabled",
            self.device, self.history_size,
        )

    def gps_callback(self, msg):
        with self.gps_lock:
            self.latest_gps = msg

    def status_callback(self, msg):
        with self.status_lock:
            self.latest_vehicle_status = msg

    def map_callback(self, msg):
        cfg = self.data_cfg
        try:
            value, _mask = mgeo_tokens_from_msg(
                msg, cfg["max_map_tokens"], cfg["map_token_dim"], True
            )
        except (ValueError, TypeError) as error:
            rospy.logwarn_throttle(2.0, "Invalid V2 MGeo tokens: %s", error)
            return
        with self.static_lock:
            self.map_input = value
            self.static_revision += 1

    def route_callback(self, msg):
        cfg = self.data_cfg
        try:
            value, _mask = route_tokens_from_msg(
                msg, cfg["max_route_tokens"], 50.0,
                cfg.get("local_route_frame", "base_link"), True,
            )
        except (ValueError, TypeError) as error:
            rospy.logwarn_throttle(2.0, "Invalid V2 route: %s", error)
            return
        with self.static_lock:
            self.route_input, self.route_msg = value, msg
            self.static_revision += 1

    def tensor(self, value):
        return torch.from_numpy(value).unsqueeze(0).to(self.device)

    def smooth_absolute_prediction(self, prediction):
        """Smooth model-local absolute future x/y without cumulative summation."""
        raw_positions = np.vstack((
            np.zeros((1, 2), dtype=np.float64),
            np.asarray(prediction[:, :2], dtype=np.float64),
        ))
        smoothed = self.smooth_positions(
            raw_positions, self.smoothing_strength
        )
        positions = self.interpolate_positions(
            smoothed, self.points_per_segment
        )
        source_index = np.arange(len(raw_positions), dtype=np.float64)
        target_index = np.linspace(
            0.0, len(raw_positions) - 1, len(positions)
        )
        source_speed = np.concatenate((
            [prediction[0, 3]],
            prediction[:, 3],
        ))
        speeds = np.interp(target_index, source_index, source_speed)
        yaws = self.path_yaws(
            positions, fallback_yaw=float(prediction[0, 2])
        )
        return (
            positions.astype(np.float32),
            yaws.astype(np.float32),
            speeds.astype(np.float32),
        )

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

    def dynamic_callback(self, front_msg, left_msg, right_msg, lidar_msg, odom_msg, imu_msg):
        stamp = odom_msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
        if stamp - self.last_encoded_stamp < self.sample_period - 1e-4:
            return
        cfg = self.data_cfg
        with self.gps_lock:
            gps_msg = self.latest_gps
        with self.status_lock:
            status_msg = self.latest_vehicle_status
        with self.static_lock:
            route_msg = self.route_msg
        try:
            front = image_from_msg(front_msg, cfg["front_image_width"], cfg["front_image_height"])
            left = image_from_msg(left_msg, cfg["side_image_width"], cfg["side_image_height"])
            right = image_from_msg(right_msg, cfg["side_image_width"], cfg["side_image_height"])
            lidar = points_to_bev(
                pointcloud2_to_xyz(lidar_msg), cfg["bev_x_range"], cfg["bev_y_range"],
                cfg["bev_height"], cfg["bev_width"],
            )
            ego = self.weighted_ego_vector(
                odom_msg, imu_msg, status_msg, gps_msg, stamp, route_msg
            )
        except (ValueError, TypeError) as error:
            rospy.logwarn_throttle(2.0, "Invalid dynamic V2 input: %s", error)
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

            mgeo, route = static_cache
            output = self.model(*history, mgeo, route)
            mode = int(torch.argmax(output["mode_logits"][0]).item())
            trajectory = output["trajectory"][0, mode].float().cpu().numpy()
            trajectory[:, :2] *= 50.0
            trajectory[:, 2] *= np.pi
            # The trained speed target is normalized m/s.
            trajectory[:, 3] *= 20.0
            positions, yaws, speeds = self.smooth_absolute_prediction(
                trajectory
            )
            smoothed = np.column_stack((positions, yaws, speeds))
            cpu_output = {
                "trajectory": smoothed,
                "mode_scores": torch.softmax(output["mode_logits"][0], dim=-1).float().cpu().numpy(),
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
            mode_scores = self.latest_output["mode_scores"].copy()
            update_time, prediction_stamp = self.latest_prediction_time, self.latest_prediction_stamp
        if (rospy.Time.now() - update_time).to_sec() > self.prediction_timeout:
            rospy.logwarn_throttle(2.0, "Planner V2 prediction stale; publication paused")
            return

        path = Path()
        path.header.stamp = prediction_stamp
        path.header.frame_id = "base_link"
        for point in trajectory:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(point[0]), float(point[1])
            pose.pose.orientation.z = math.sin(float(point[2]) * 0.5)
            pose.pose.orientation.w = math.cos(float(point[2]) * 0.5)
            path.poses.append(pose)
        speeds_kph = np.maximum(trajectory[:, 3], 0.0) * 3.6
        self.speed_publisher.publish(Float32MultiArray(data=speeds_kph.tolist()))
        if len(trajectory):
            predicted_speeds_mps = np.maximum(trajectory[:, 3], 0.0)
            target_speed_mps = float(np.max(predicted_speeds_mps))
            target_speed_kph = target_speed_mps * 3.6
            self.mpc_speed_publisher.publish(Float32(data=target_speed_mps))
            self.kph_speed_publisher.publish(Float32(data=target_speed_kph))
            rospy.loginfo_throttle(
                1.0,
                "Planner target speed: %.3f m/s (%.2f km/h, horizon max; range %.3f-%.3f m/s)",
                target_speed_mps,
                target_speed_kph,
                float(np.min(predicted_speeds_mps)),
                float(np.max(predicted_speeds_mps)),
            )
        self.path_publisher.publish(path)
        self.mode_publisher.publish(Float32MultiArray(data=mode_scores.tolist()))


if __name__ == "__main__":
    rospy.init_node("multimodal_inference")
    InferenceNode()
    rospy.spin()
