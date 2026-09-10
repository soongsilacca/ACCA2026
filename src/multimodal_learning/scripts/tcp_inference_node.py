#!/usr/bin/env python3
"""ROS inference bridge for the TCP MORAI trajectory checkpoint."""

import math
import os
import sys
import threading

import cv2
import numpy as np
import rospy
import torch
import torch.nn as nn
from geometry_msgs.msg import Point, PoseStamped
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from nav_msgs.msg import Odometry, Path
from scipy.interpolate import PchipInterpolator
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import ColorRGBA, Float32, Float32MultiArray, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

# Importing multimodal_learning executes its package initializer, which uses
# the bundled V9 lineage. Make that lineage visible before the package import.
_PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_MODEL_ARTIFACT_DIR = os.path.join(_PACKAGE_DIR, "model_artifacts")
for _source_dir in (_MODEL_ARTIFACT_DIR, "/home/acca"):
    if _source_dir not in sys.path:
        sys.path.insert(0, _source_dir)

from multimodal_learning.sensor_preprocess import image_from_msg


class TCPInferenceNode:
    def __init__(self):
        rospy.init_node("multimodal_inference", anonymous=False)

        artifact_zip = os.path.abspath(rospy.get_param(
            "~artifact_zip", "/home/acca/tcp_morai_finetune.zip"
        ))
        checkpoint_path = os.path.abspath(rospy.get_param(
            "~checkpoint_path", "/home/acca/latest.pt"
        ))
        for path in (artifact_zip, checkpoint_path):
            if not os.path.isfile(path):
                raise FileNotFoundError(path)
        if artifact_zip not in sys.path:
            sys.path.insert(0, artifact_zip)
        from tcp_morai_finetune.model import TCPMorai

        class TCPMoraiState(TCPMorai):
            def __init__(self):
                super().__init__()
                self.state_head = nn.Sequential(
                    nn.Linear(1000, 256),
                    nn.ReLU(inplace=True),
                    nn.Dropout(p=0.2),
                    nn.Linear(256, 3),
                )

            def forward(self, image, state, target_point):
                feature_embedding, _ = self._perception_forward(image)
                measurement_feature = self.measurements(state)
                hidden = self.join_traj(torch.cat(
                    (feature_embedding, measurement_feature), dim=1
                ))
                waypoint = torch.zeros(
                    (hidden.shape[0], 2), device=hidden.device,
                    dtype=hidden.dtype,
                )
                predictions = []
                for _ in range(self.config.pred_len):
                    hidden = self.decoder_traj(
                        torch.cat((waypoint, target_point), dim=1), hidden
                    )
                    waypoint = waypoint + self.output_traj(hidden)
                    predictions.append(waypoint)
                logits = self.state_head(feature_embedding)
                return {
                    "waypoints": torch.stack(predictions, dim=1),
                    "speed": self.speed_branch(feature_embedding),
                    "action_logits": logits,
                    "action_probabilities": torch.softmax(logits, dim=-1),
                }

            def forward_full_policy(self, image, state, target_point):
                """Run the TCP trajectory, state, and direct-control heads."""
                feature_embedding, spatial = self._perception_forward(image)
                measurement_feature = self.measurements(state)
                action_logits = self.state_head(feature_embedding)

                trajectory_feature = self.join_traj(torch.cat(
                    (feature_embedding, measurement_feature), dim=1
                ))
                waypoint = torch.zeros(
                    (trajectory_feature.shape[0], 2),
                    device=trajectory_feature.device,
                    dtype=trajectory_feature.dtype,
                )
                hidden = trajectory_feature
                predictions = []
                trajectory_hidden = []
                for _ in range(self.config.pred_len):
                    hidden = self.decoder_traj(
                        torch.cat((waypoint, target_point), dim=1), hidden
                    )
                    trajectory_hidden.append(hidden)
                    waypoint = waypoint + self.output_traj(hidden)
                    predictions.append(waypoint)
                trajectory_hidden = torch.stack(trajectory_hidden, dim=1)

                # The original TCP control attention expects the 8x29 feature
                # map produced by a 256x900 input image.
                initial_attention = self.init_att(measurement_feature).view(
                    -1, 1, 8, 29
                )
                attended = torch.sum(
                    spatial * initial_attention, dim=(2, 3)
                )
                control_feature = self.join_ctrl(torch.cat(
                    (attended, measurement_feature), dim=1
                ))
                policy = self.policy_head(control_feature)
                action_alpha = self.dist_mu(policy)
                action_beta = self.dist_sigma(policy)

                return {
                    "waypoints": torch.stack(predictions, dim=1),
                    "speed": self.speed_branch(feature_embedding),
                    "action_logits": action_logits,
                    "action_probabilities": torch.softmax(
                        action_logits, dim=-1
                    ),
                    "control_alpha": action_alpha,
                    "control_beta": action_beta,
                }

        self.device = torch.device(
            "cuda"
            if torch.cuda.is_available() and rospy.get_param("~use_cuda", True)
            else "cpu"
        )
        checkpoint = torch.load(
            checkpoint_path, map_location="cpu", weights_only=True
        )
        self.checkpoint_schema = checkpoint.get("schema")
        if self.checkpoint_schema not in {
            "tcp_morai_trajectory_v1",
            "tcp_morai_trajectory_state_v2",
            "tcp_morai_trajectory_state_v3",
            "tcp_morai_full_policy_v1",
            "tcp_original_trajectory_v1",
        }:
            raise ValueError(
                "unsupported TCP checkpoint schema: %s"
                % self.checkpoint_schema
            )
        self.has_state_head = self.checkpoint_schema in {
            "tcp_morai_trajectory_state_v2",
            "tcp_morai_trajectory_state_v3",
            "tcp_morai_full_policy_v1",
        }
        self.state_head_is_action = self.checkpoint_schema in {
            "tcp_morai_trajectory_state_v2",
            "tcp_morai_trajectory_state_v3",
            "tcp_morai_full_policy_v1",
        }
        self.direct_control = bool(rospy.get_param(
            "~direct_control", False
        ))
        self.direct_steer_gain = float(rospy.get_param(
            "~direct_steer_gain", 1.0
        ))
        if self.direct_steer_gain <= 0.0:
            raise ValueError("direct_steer_gain must be positive")
        if (
            self.direct_control
            and self.checkpoint_schema != "tcp_morai_full_policy_v1"
        ):
            raise ValueError(
                "direct_control requires tcp_morai_full_policy_v1"
            )
        self.model = (
            TCPMoraiState() if self.has_state_head else TCPMorai()
        ).to(self.device)
        self.model.load_state_dict(checkpoint["model_state"], strict=True)
        self.model.eval()

        self.inference_period = float(rospy.get_param(
            "~model_inference_period", 0.3
        ))
        self.publish_hz = float(rospy.get_param(
            "~planning_publish_hz", 20.0
        ))
        self.prediction_timeout = float(rospy.get_param(
            "~prediction_timeout", 1.0
        ))
        self.path_spacing = float(rospy.get_param("~path_spacing", 0.1))
        self.velocity_scale = float(rospy.get_param(
            "~vehicle_velocity_scale", 1.0 / 3.6
        ))
        self.mgeo_target_velocity_topic = rospy.get_param(
            "~mgeo_target_velocity_topic", "/mgeo_target_velocity"
        )
        self.tcp_path_for_drive = bool(rospy.get_param(
            "~tcp_path_for_drive", False
        ))
        self.tcp_stop_speed_ratio = float(rospy.get_param(
            "~tcp_stop_speed_ratio", 0.3
        ))
        self.tcp_stop_absolute_speed = float(rospy.get_param(
            "~tcp_stop_absolute_speed_mps", 1.0
        ))
        self.stop_enter_probability = float(rospy.get_param(
            "~stop_enter_probability", 0.8
        ))
        self.stop_release_probability = float(rospy.get_param(
            "~stop_release_probability", 0.7
        ))
        self.avoid_enter_probability = float(rospy.get_param(
            "~avoid_enter_probability", 0.3
        ))
        self.state_stop_active = False
        if self.inference_period <= 0.0 or self.path_spacing <= 0.0:
            raise ValueError("inference period and path spacing must be positive")
        if not 0.0 < self.tcp_stop_speed_ratio < 1.0:
            raise ValueError("TCP stop speed ratio must be in (0, 1)")
        if self.tcp_stop_absolute_speed < 0.0:
            raise ValueError("TCP absolute stop speed cannot be negative")
        if not 0.0 <= self.stop_release_probability < self.stop_enter_probability <= 1.0:
            raise ValueError("TCP STOP thresholds require 0 <= release < enter <= 1")
        if not 0.0 <= self.avoid_enter_probability <= 1.0:
            raise ValueError("TCP AVOID threshold must be in [0, 1]")

        self.state_lock = threading.Lock()
        self.latest_odom = None
        self.latest_status = None
        self.latest_route = None
        self.latest_mgeo_speed = None
        self.last_inference_stamp = float("-inf")

        self.prediction_lock = threading.Lock()
        self.latest_prediction = None
        self.latest_prediction_time = None

        self.path_pub = rospy.Publisher(
            "/multimodal_learning/predicted_path", Path, queue_size=1
        )
        self.local_path_pub = rospy.Publisher(
            "/multimodal_learning/predicted_path_local", Path, queue_size=1
        )
        self.control_path_pub = rospy.Publisher(
            "/multimodal_learning/control_path", Path, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            "/multimodal_learning/predicted_path_markers",
            MarkerArray,
            queue_size=1,
        )
        self.speed_profile_pub = rospy.Publisher(
            "/multimodal_learning/target_speeds",
            Float32MultiArray,
            queue_size=1,
        )
        self.target_speed_pub = rospy.Publisher(
            "/target_velocity", Float32, queue_size=1
        )
        self.target_speed_kph_pub = rospy.Publisher(
            "/target_velocity_kph", Float32, queue_size=1
        )
        self.tcp_speed_pub = rospy.Publisher(
            "/multimodal_learning/tcp_predicted_speed",
            Float32,
            queue_size=1,
        )
        self.tcp_speed_head_pub = rospy.Publisher(
            "/multimodal_learning/tcp_speed_head",
            Float32,
            queue_size=1,
        )
        self.mode_pub = rospy.Publisher(
            "/multimodal_learning/mode_scores",
            Float32MultiArray,
            queue_size=1,
        )
        self.action_pub = rospy.Publisher(
            "/multimodal_learning/action_scores",
            Float32MultiArray,
            queue_size=1,
        )
        self.runtime_action_pub = rospy.Publisher(
            "/multimodal_learning/runtime_action", String, queue_size=1
        )
        self.direct_control_pub = rospy.Publisher(
            "/ctrl_cmd", CtrlCmd, queue_size=1
        ) if self.direct_control else None
        self.direct_control_values_pub = rospy.Publisher(
            "/multimodal_learning/tcp_control", Float32MultiArray,
            queue_size=1,
        )

        rospy.Subscriber(
            "/localization/kinematic_state",
            Odometry,
            self.odom_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/morai/ego_vehicle_status",
            EgoVehicleStatus,
            self.status_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/local_route", Path, self.route_callback, queue_size=1
        )
        rospy.Subscriber(
            self.mgeo_target_velocity_topic,
            Float32,
            self.mgeo_speed_callback,
            queue_size=1,
        )
        camera_topic = rospy.get_param(
            "~camera_topic", "/camera/front/image/compressed"
        )
        camera_type = (
            CompressedImage
            if camera_topic.rstrip("/").endswith("/compressed")
            else Image
        )
        rospy.Subscriber(
            camera_topic, camera_type, self.camera_callback, queue_size=1
        )
        self.publish_timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_hz, 1.0)),
            self.publish_prediction,
        )
        rospy.loginfo(
            "TCP MORAI checkpoint loaded on %s: %s epoch %d, "
            "model period %.2fs (%.2fHz), MGeo target speed; "
            "TCP STOP at <= %.2f m/s or below %.0f%% of current speed",
            self.device,
            checkpoint_path,
            int(checkpoint.get("epoch", -1)) + 1,
            self.inference_period,
            1.0 / self.inference_period,
            self.tcp_stop_absolute_speed,
            self.tcp_stop_speed_ratio * 100.0,
        )

    def odom_callback(self, msg):
        with self.state_lock:
            self.latest_odom = msg

    def status_callback(self, msg):
        with self.state_lock:
            self.latest_status = msg

    def route_callback(self, msg):
        with self.state_lock:
            self.latest_route = msg

    def mgeo_speed_callback(self, msg):
        speed = float(msg.data)
        if not math.isfinite(speed):
            rospy.logwarn_throttle(2.0, "Ignoring non-finite MGeo speed")
            return
        with self.state_lock:
            self.latest_mgeo_speed = max(speed, 0.0)

    @staticmethod
    def route_in_base_link(route, odom):
        if route is None or len(route.poses) < 2:
            raise ValueError("TCP requires a non-empty Local Route")
        points = np.asarray([
            [pose.pose.position.x, pose.pose.position.y]
            for pose in route.poses
        ], dtype=np.float32)
        frame = route.header.frame_id.lstrip("/")
        if frame in ("base_link", "base_footprint"):
            return points
        if frame != "map":
            raise ValueError("unsupported Local Route frame: %s" % frame)
        orientation = odom.pose.pose.orientation
        yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ])[2]
        delta = points - np.asarray([
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
        ], dtype=np.float32)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return np.column_stack((
            cosine * delta[:, 0] + sine * delta[:, 1],
            -sine * delta[:, 0] + cosine * delta[:, 1],
        )).astype(np.float32)

    @staticmethod
    def route_target(route_xy, lookahead_m):
        closest = int(np.square(route_xy).sum(axis=1).argmin())
        forward = route_xy[closest:]
        if len(forward) < 2:
            return route_xy[-1].copy()
        segment = np.linalg.norm(np.diff(forward, axis=0), axis=1)
        cumulative = np.concatenate((
            np.zeros(1, dtype=np.float32), np.cumsum(segment)
        ))
        index = int(np.searchsorted(cumulative, lookahead_m, side="left"))
        return forward[min(index, len(forward) - 1)].copy()

    @classmethod
    def route_command(cls, route_xy):
        target = cls.route_target(route_xy, 20.0)
        angle = math.atan2(float(target[1]), max(float(target[0]), 1.0e-3))
        command = 3
        if angle > math.radians(12.0):
            command = 0
        elif angle < -math.radians(12.0):
            command = 1
        one_hot = np.zeros(6, dtype=np.float32)
        one_hot[command] = 1.0
        return one_hot

    @staticmethod
    def morai_to_tcp(point):
        return np.asarray([-point[1], -point[0]], dtype=np.float32)

    @staticmethod
    def tcp_to_morai(points):
        value = np.asarray(points, dtype=np.float32)
        return np.column_stack((-value[:, 1], -value[:, 0])).astype(np.float32)

    @staticmethod
    def preprocess_image(image_chw, direct_control=False):
        image = np.transpose(image_chw, (1, 2, 0))
        if direct_control:
            image = cv2.resize(
                image, (900, 256), interpolation=cv2.INTER_AREA
            )
        else:
            height, width = image.shape[:2]
            crop = min(height, width)
            top = (height - crop) // 2
            left = (width - crop) // 2
            image = image[top:top + crop, left:left + crop]
            image = cv2.resize(
                image, (256, 256), interpolation=cv2.INTER_AREA
            )
        value = image.astype(np.float32) / 255.0
        mean = np.asarray([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.asarray([0.229, 0.224, 0.225], dtype=np.float32)
        value = (value - mean[None, None, :]) / std[None, None, :]
        return np.ascontiguousarray(value.transpose(2, 0, 1))

    @staticmethod
    def beta_policy_action(alpha, beta):
        alpha = np.maximum(np.asarray(alpha, dtype=np.float32), 1.0e-4)
        beta = np.maximum(np.asarray(beta, dtype=np.float32), 1.0e-4)
        return np.clip(alpha / (alpha + beta) * 2.0 - 1.0, -1.0, 1.0)

    def dense_path(self, waypoints):
        points = np.asarray(waypoints, dtype=np.float64)
        segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(segment)))
        keep = np.r_[True, np.diff(cumulative) > 1.0e-4]
        points = points[keep]
        cumulative = cumulative[keep]
        if len(points) < 2 or cumulative[-1] < self.path_spacing:
            raise ValueError("TCP produced a degenerate waypoint path")
        station = np.arange(
            0.0,
            cumulative[-1] + self.path_spacing * 0.5,
            self.path_spacing,
            dtype=np.float64,
        )
        if station[-1] < cumulative[-1] - 1.0e-4:
            station = np.append(station, cumulative[-1])
        x = PchipInterpolator(cumulative, points[:, 0])(station)
        y = PchipInterpolator(cumulative, points[:, 1])(station)
        xy = np.column_stack((x, y))
        derivative = np.gradient(xy, station, axis=0, edge_order=1)
        yaw = np.arctan2(derivative[:, 1], derivative[:, 0])
        return xy.astype(np.float32), yaw.astype(np.float32)

    @staticmethod
    def waypoint_target_speed(waypoints):
        """Estimate future speed only from TCP's four trajectory outputs."""
        horizons = np.asarray([0.6, 1.0, 1.6, 2.0], dtype=np.float64)
        points = np.asarray(waypoints, dtype=np.float64)
        segment_speed = (
            np.linalg.norm(np.diff(points, axis=0), axis=1)
            / np.diff(horizons)
        )
        if not np.isfinite(segment_speed).all():
            raise ValueError("TCP waypoint speed is not finite")
        # The median rejects one malformed waypoint interval without adding a
        # road-speed source or a state-dependent mode switch.
        return max(float(np.median(segment_speed)), 0.0)

    def tcp_stop_active(self, waypoint_speed, current_speed):
        ratio_threshold = (
            max(float(current_speed), 0.0) * self.tcp_stop_speed_ratio
        )
        waypoint_speed = float(waypoint_speed)
        stop_active = (
            waypoint_speed <= self.tcp_stop_absolute_speed
            or waypoint_speed < ratio_threshold
        )
        return (
            stop_active,
            max(self.tcp_stop_absolute_speed, ratio_threshold),
        )

    def publish_tcp_mode(
        self, stop_active, action_probabilities=None, action_index=None
    ):
        if action_probabilities is None:
            action_probabilities = [0.0, 1.0, 0.0] if stop_active else [1.0, 0.0, 0.0]
        if stop_active:
            self.mode_pub.publish(Float32MultiArray(data=[1.0, 0.0]))
            self.runtime_action_pub.publish(String("TCP_STOP"))
        else:
            self.mode_pub.publish(Float32MultiArray(data=[0.0, 1.0]))
            action = (
                int(np.argmax(action_probabilities))
                if action_index is None else int(action_index)
            )
            self.runtime_action_pub.publish(String(
                "TCP_AVOID" if action == 2 else "TCP_DRIVE"
            ))
        self.action_pub.publish(Float32MultiArray(
            data=np.asarray(action_probabilities, dtype=np.float32).tolist()
        ))

    def camera_callback(self, msg):
        stamp = msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
        if stamp - self.last_inference_stamp < self.inference_period - 1.0e-4:
            return
        with self.state_lock:
            odom = self.latest_odom
            status = self.latest_status
            route = self.latest_route
            mgeo_speed = self.latest_mgeo_speed
        if odom is None or route is None:
            rospy.logwarn_throttle(
                2.0, "TCP waiting for odometry and Local Route"
            )
            return
        try:
            route_xy = self.route_in_base_link(route, odom)
            target_morai = self.route_target(route_xy, 10.0)
            target_tcp = self.morai_to_tcp(target_morai)
            command = self.route_command(route_xy)
            if status is None:
                speed_mps = abs(float(odom.twist.twist.linear.x))
            else:
                speed_mps = math.hypot(
                    float(status.velocity.x), float(status.velocity.y)
                ) * self.velocity_scale
            state = np.concatenate((
                np.asarray([speed_mps / 12.0], dtype=np.float32),
                target_tcp,
                command,
            ))
            image = image_from_msg(msg, 640, 360)
            image = self.preprocess_image(image, self.direct_control)
            with torch.no_grad():
                image_tensor = torch.from_numpy(image).unsqueeze(0).to(
                    self.device
                )
                state_tensor = torch.from_numpy(state).unsqueeze(0).to(
                    self.device
                )
                target_tensor = torch.from_numpy(target_tcp).unsqueeze(0).to(
                    self.device
                )
                output = (
                    self.model.forward_full_policy(
                        image_tensor, state_tensor, target_tensor
                    )
                    if self.direct_control else self.model(
                        image_tensor, state_tensor, target_tensor
                    )
                )
            action_probabilities = (
                output["action_probabilities"][0].float().cpu().numpy()
                if self.state_head_is_action else None
            )
            waypoints_tcp = output["waypoints"][0].float().cpu().numpy()
            waypoints_local = self.tcp_to_morai(waypoints_tcp)
            tcp_waypoint_speed = self.waypoint_target_speed(waypoints_local)
            stop_active, stop_speed_threshold = self.tcp_stop_active(
                tcp_waypoint_speed, speed_mps
            )
            if action_probabilities is not None:
                stop_probability = float(action_probabilities[1])
                if self.state_stop_active:
                    self.state_stop_active = (
                        stop_probability >= self.stop_release_probability
                    )
                else:
                    self.state_stop_active = (
                        stop_probability >= self.stop_enter_probability
                    )
                stop_active = self.state_stop_active
                stop_speed_threshold = 0.0
                action_index = (
                    1 if stop_active else
                    2 if float(action_probabilities[2])
                    >= self.avoid_enter_probability else 0
                )
            else:
                action_index = 1 if stop_active else 0
            tcp_speed_head = (
                float(output["speed"][0, 0].float().cpu()) * 12.0
            )
            direct_control = None
            if self.direct_control:
                direct_control = self.beta_policy_action(
                    output["control_alpha"][0].float().cpu().numpy(),
                    output["control_beta"][0].float().cpu().numpy(),
                )
            try:
                local_xy, local_yaw = self.dense_path(waypoints_local)
            except ValueError as error:
                # A stationary TCP output has no arc length to interpolate.
                # A stopped trajectory has no arc length. Retain the last
                # valid geometry while publishing the TCP STOP decision.
                with self.prediction_lock:
                    previous = self.latest_prediction
                self.last_inference_stamp = stamp
                self.publish_tcp_mode(
                    stop_active, action_probabilities, action_index
                )
                target_speed = 0.0 if stop_active else (
                    0.0 if mgeo_speed is None else mgeo_speed
                )
                self.target_speed_pub.publish(Float32(target_speed))
                self.target_speed_kph_pub.publish(Float32(target_speed * 3.6))
                self.tcp_speed_pub.publish(Float32(tcp_waypoint_speed))
                self.tcp_speed_head_pub.publish(Float32(tcp_speed_head))
                if previous is None:
                    rospy.logwarn_throttle(
                        2.0,
                        "%s; TCP %s applied while waiting for the first "
                        "valid path (target %.2f m/s)",
                        error,
                        "STOP" if stop_active else "DRIVE",
                        target_speed,
                    )
                    return
                prediction = dict(previous)
                prediction.update({
                    "stamp": rospy.Time.from_sec(stamp),
                    "target_speed": target_speed,
                    "mgeo_speed": 0.0 if mgeo_speed is None else mgeo_speed,
                    "tcp_waypoint_speed": tcp_waypoint_speed,
                    "tcp_speed_head": tcp_speed_head,
                    "current_speed": speed_mps,
                    "stop_speed_threshold": stop_speed_threshold,
                    "stop_active": stop_active,
                    "action_index": action_index,
                    "action_probabilities": action_probabilities,
                    "direct_control": direct_control,
                })
                with self.prediction_lock:
                    self.latest_prediction = prediction
                    self.latest_prediction_time = rospy.Time.now()
                rospy.logwarn_throttle(
                    2.0,
                    "%s; retaining the last valid path under TCP %s",
                    error,
                    "STOP" if stop_active else "DRIVE",
                )
                return
        except (ValueError, TypeError, RuntimeError) as error:
            rospy.logerr_throttle(2.0, "TCP inference failed: %s", error)
            return

        target_speed = 0.0 if stop_active else (
            0.0 if mgeo_speed is None else mgeo_speed
        )

        orientation = odom.pose.pose.orientation
        vehicle_yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ])[2]
        cosine, sine = math.cos(vehicle_yaw), math.sin(vehicle_yaw)
        map_xy = np.column_stack((
            odom.pose.pose.position.x
            + cosine * local_xy[:, 0] - sine * local_xy[:, 1],
            odom.pose.pose.position.y
            + sine * local_xy[:, 0] + cosine * local_xy[:, 1],
        ))
        map_yaw = np.arctan2(
            np.sin(local_yaw + vehicle_yaw),
            np.cos(local_yaw + vehicle_yaw),
        )
        raw_map_xy = np.column_stack((
            odom.pose.pose.position.x
            + cosine * waypoints_local[:, 0]
            - sine * waypoints_local[:, 1],
            odom.pose.pose.position.y
            + sine * waypoints_local[:, 0]
            + cosine * waypoints_local[:, 1],
        ))
        prediction = {
            "stamp": rospy.Time.from_sec(stamp),
            "map_xy": map_xy.astype(np.float32),
            "map_yaw": map_yaw.astype(np.float32),
            "local_xy": local_xy,
            "local_yaw": local_yaw,
            "raw_map_xy": raw_map_xy.astype(np.float32),
            "target_speed": target_speed,
            "mgeo_speed": 0.0 if mgeo_speed is None else mgeo_speed,
            "tcp_waypoint_speed": tcp_waypoint_speed,
            "tcp_speed_head": tcp_speed_head,
            "current_speed": speed_mps,
            "stop_speed_threshold": stop_speed_threshold,
            "stop_active": stop_active,
            "action_index": action_index,
            "action_probabilities": action_probabilities,
            "direct_control": direct_control,
        }
        with self.prediction_lock:
            self.latest_prediction = prediction
            self.latest_prediction_time = rospy.Time.now()
        self.last_inference_stamp = stamp
        rospy.loginfo_throttle(
            1.0,
            "TCP path: %d points, %.2fm, %s, target %.1f km/h, "
            "TCP waypoint %.1f km/h < stop threshold %.1f km/h "
            "(current %.1f km/h; head %.1f km/h)",
            len(local_xy),
            float(np.linalg.norm(np.diff(local_xy, axis=0), axis=1).sum()),
            "STOP" if stop_active else "DRIVE",
            target_speed * 3.6,
            tcp_waypoint_speed * 3.6,
            stop_speed_threshold * 3.6,
            speed_mps * 3.6,
            tcp_speed_head * 3.6,
        )

    def publish_direct_control(self, prediction=None):
        if self.direct_control_pub is None:
            return
        command = CtrlCmd()
        command.ctrl_mode = 2
        command.cmd_type = 1
        command.gear = 4
        if prediction is None or prediction.get("direct_control") is None:
            command.accel = 0.0
            command.brake = 1.0
            command.steer = 0.0
            values = [0.0, 1.0, 0.0]
        else:
            signed_accel, normalized_steer = np.asarray(
                prediction["direct_control"], dtype=np.float32
            )
            command.accel = float(np.clip(signed_accel, 0.0, 1.0))
            command.brake = float(np.clip(-signed_accel, 0.0, 1.0))
            command.steer = math.radians(float(np.clip(
                normalized_steer * self.direct_steer_gain,
                -1.0,
                1.0,
            ) * 40.0))
            values = [command.accel, command.brake, command.steer]
        self.direct_control_pub.publish(command)
        self.direct_control_values_pub.publish(Float32MultiArray(data=values))
        rospy.loginfo_throttle(
            1.0,
            "TCP direct control: accel=%.3f brake=%.3f steer=%.2f deg",
            command.accel,
            command.brake,
            math.degrees(command.steer),
        )

    @staticmethod
    def make_path(xy, yaw, stamp, frame):
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = frame
        for point, angle in zip(xy, yaw):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            quaternion = quaternion_from_euler(0.0, 0.0, float(angle))
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            path.poses.append(pose)
        return path

    def make_markers(self, prediction):
        markers = MarkerArray()
        stamp = prediction["stamp"]
        map_xy = np.asarray(prediction["map_xy"], dtype=np.float32)
        map_yaw = np.asarray(prediction["map_yaw"], dtype=np.float32)
        lifetime = rospy.Duration(1.0)

        def speed_color(speed_mps, alpha):
            ratio = float(np.clip(speed_mps / 13.89, 0.0, 1.0))
            return ColorRGBA(
                r=0.45 + 0.55 * ratio,
                g=0.05 * (1.0 - ratio),
                b=0.95 * (1.0 - ratio),
                a=alpha,
            )

        surface = Marker()
        surface.header.frame_id = "map"
        surface.header.stamp = stamp
        surface.ns = "tcp_predicted_path"
        surface.id = 0
        surface.type = Marker.TRIANGLE_LIST
        surface.action = Marker.ADD
        surface.pose.orientation.w = 1.0
        surface.scale.x = surface.scale.y = surface.scale.z = 1.0
        surface.color.a = 0.0
        surface.lifetime = lifetime

        width = 1.8
        color = speed_color(prediction["target_speed"], 0.55)
        if len(map_xy) >= 2:
            left = []
            right = []
            for point, yaw in zip(map_xy, map_yaw):
                nx = -math.sin(float(yaw))
                ny = math.cos(float(yaw))
                left.append((point[0] + nx * width * 0.5,
                             point[1] + ny * width * 0.5))
                right.append((point[0] - nx * width * 0.5,
                              point[1] - ny * width * 0.5))
            for idx in range(len(map_xy) - 1):
                quad = (
                    left[idx], right[idx], left[idx + 1],
                    right[idx], right[idx + 1], left[idx + 1],
                )
                for x, y in quad:
                    surface.points.append(Point(float(x), float(y), 0.12))
                    surface.colors.append(color)
            markers.markers.append(surface)

        line = Marker()
        line.header = surface.header
        line.ns = surface.ns
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.22
        line.color = speed_color(prediction["target_speed"], 1.0)
        line.lifetime = lifetime
        for row in map_xy:
            line.points.append(Point(float(row[0]), float(row[1]), 0.24))
        markers.markers.append(line)

        text = Marker()
        text.header = surface.header
        text.ns = surface.ns
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.orientation.w = 1.0
        text.pose.position.x = float(map_xy[0, 0])
        text.pose.position.y = float(map_xy[0, 1])
        text.pose.position.z = 1.5
        text.scale.z = 0.7
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        probabilities = prediction.get("action_probabilities")
        if probabilities is None:
            probabilities = [
                0.0 if prediction["stop_active"] else 1.0,
                1.0 if prediction["stop_active"] else 0.0,
                0.0,
            ]
        probabilities = np.asarray(probabilities, dtype=np.float32)
        action = int(prediction.get("action_index", np.argmax(probabilities)))
        action_name = ("DRIVE", "STOP", "AVOID")[action]
        path_source = (
            "TCP" if (self.tcp_path_for_drive or action == 2)
            and not prediction["stop_active"] else "MGeo"
        )
        text.text = (
            "%s | path %s | D %.2f  S %.2f  A %.2f | "
            "target %.1f | wp %.1f km/h"
        ) % (
            action_name,
            path_source,
            float(probabilities[0]),
            float(probabilities[1]),
            float(probabilities[2]),
            prediction["target_speed"] * 3.6,
            prediction["tcp_waypoint_speed"] * 3.6,
        )
        text.lifetime = lifetime
        markers.markers.append(text)
        return markers

    def publish_prediction(self, _event):
        with self.prediction_lock:
            prediction = self.latest_prediction
            prediction_time = self.latest_prediction_time
        if prediction is None:
            self.publish_direct_control()
            return
        if (
            prediction_time is None
            or (rospy.Time.now() - prediction_time).to_sec()
            > self.prediction_timeout
        ):
            self.target_speed_pub.publish(Float32(0.0))
            self.target_speed_kph_pub.publish(Float32(0.0))
            self.publish_direct_control()
            rospy.logwarn_throttle(2.0, "TCP prediction stale; safe stop")
            return

        map_path = self.make_path(
            prediction["map_xy"],
            prediction["map_yaw"],
            prediction["stamp"],
            "map",
        )
        local_path = self.make_path(
            prediction["local_xy"],
            prediction["local_yaw"],
            prediction["stamp"],
            "base_link",
        )
        target = prediction["target_speed"]
        self.path_pub.publish(map_path)
        self.local_path_pub.publish(local_path)
        action = int(prediction.get(
            "action_index", 1 if prediction["stop_active"] else 0
        ))
        if ((self.tcp_path_for_drive or action == 2)
                and not prediction["stop_active"]):
            self.control_path_pub.publish(map_path)
        else:
            with self.state_lock:
                route = self.latest_route
            if route is not None and len(route.poses) >= 2:
                self.control_path_pub.publish(route)
        self.marker_pub.publish(self.make_markers(prediction))
        self.speed_profile_pub.publish(Float32MultiArray(
            data=np.full(len(map_path.poses), target, dtype=np.float32).tolist()
        ))
        self.target_speed_pub.publish(Float32(target))
        self.target_speed_kph_pub.publish(Float32(target * 3.6))
        self.tcp_speed_pub.publish(Float32(prediction["tcp_waypoint_speed"]))
        self.tcp_speed_head_pub.publish(Float32(prediction["tcp_speed_head"]))
        self.publish_tcp_mode(
            prediction["stop_active"],
            prediction.get("action_probabilities"),
            prediction.get("action_index"),
        )
        self.publish_direct_control(prediction)


if __name__ == "__main__":
    try:
        TCPInferenceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
