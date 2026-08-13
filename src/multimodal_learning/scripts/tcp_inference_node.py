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
from geometry_msgs.msg import Point, PoseStamped
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry, Path
from scipy.interpolate import PchipInterpolator
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Float32MultiArray, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

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

        self.device = torch.device(
            "cuda"
            if torch.cuda.is_available() and rospy.get_param("~use_cuda", True)
            else "cpu"
        )
        checkpoint = torch.load(
            checkpoint_path, map_location="cpu", weights_only=True
        )
        if checkpoint.get("schema") != "tcp_morai_trajectory_v1":
            raise ValueError(
                "checkpoint is not tcp_morai_trajectory_v1: %s"
                % checkpoint.get("schema")
            )
        self.model = TCPMorai().to(self.device)
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
        self.tcp_stop_speed_ratio = float(rospy.get_param(
            "~tcp_stop_speed_ratio", 0.3
        ))
        self.tcp_stop_absolute_speed = float(rospy.get_param(
            "~tcp_stop_absolute_speed_mps", 1.0
        ))
        if self.inference_period <= 0.0 or self.path_spacing <= 0.0:
            raise ValueError("inference period and path spacing must be positive")
        if not 0.0 < self.tcp_stop_speed_ratio < 1.0:
            raise ValueError("TCP stop speed ratio must be in (0, 1)")
        if self.tcp_stop_absolute_speed < 0.0:
            raise ValueError("TCP absolute stop speed cannot be negative")

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
    def center_crop_resize(image_chw, size=256):
        image = np.transpose(image_chw, (1, 2, 0))
        height, width = image.shape[:2]
        crop = min(height, width)
        top = (height - crop) // 2
        left = (width - crop) // 2
        image = image[top:top + crop, left:left + crop]
        image = cv2.resize(image, (size, size), interpolation=cv2.INTER_AREA)
        value = image.astype(np.float32) / 255.0
        mean = np.asarray([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.asarray([0.229, 0.224, 0.225], dtype=np.float32)
        value = (value - mean[None, None, :]) / std[None, None, :]
        return np.ascontiguousarray(value.transpose(2, 0, 1))

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

    def publish_tcp_mode(self, stop_active):
        if stop_active:
            self.mode_pub.publish(Float32MultiArray(data=[1.0, 0.0]))
            self.action_pub.publish(Float32MultiArray(data=[0.0, 1.0, 0.0]))
            self.runtime_action_pub.publish(String("TCP_STOP"))
        else:
            self.mode_pub.publish(Float32MultiArray(data=[0.0, 1.0]))
            self.action_pub.publish(Float32MultiArray(data=[1.0, 0.0, 0.0]))
            self.runtime_action_pub.publish(String("TCP_DRIVE"))

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
            image = self.center_crop_resize(image)
            with torch.no_grad():
                output = self.model(
                    torch.from_numpy(image).unsqueeze(0).to(self.device),
                    torch.from_numpy(state).unsqueeze(0).to(self.device),
                    torch.from_numpy(target_tcp).unsqueeze(0).to(self.device),
                )
            waypoints_tcp = output["waypoints"][0].float().cpu().numpy()
            waypoints_local = self.tcp_to_morai(waypoints_tcp)
            tcp_waypoint_speed = self.waypoint_target_speed(waypoints_local)
            stop_active, stop_speed_threshold = self.tcp_stop_active(
                tcp_waypoint_speed, speed_mps
            )
            tcp_speed_head = (
                float(output["speed"][0, 0].float().cpu()) * 12.0
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
                self.publish_tcp_mode(stop_active)
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
        dense = Marker()
        dense.header.frame_id = "map"
        dense.header.stamp = stamp
        dense.ns = "tcp_predicted_path"
        dense.id = 0
        dense.type = Marker.LINE_STRIP
        dense.action = Marker.ADD
        dense.pose.orientation.w = 1.0
        dense.scale.x = 0.15
        dense.color.r = 0.0
        dense.color.g = 0.9
        dense.color.b = 1.0
        dense.color.a = 1.0
        dense.lifetime = rospy.Duration(1.0)
        for row in prediction["map_xy"]:
            dense.points.append(Point(float(row[0]), float(row[1]), 0.15))
        markers.markers.append(dense)

        raw = Marker()
        raw.header = dense.header
        raw.ns = dense.ns
        raw.id = 1
        raw.type = Marker.SPHERE_LIST
        raw.action = Marker.ADD
        raw.pose.orientation.w = 1.0
        raw.scale.x = raw.scale.y = raw.scale.z = 0.45
        raw.color.r = 1.0
        raw.color.g = 0.1
        raw.color.b = 0.8
        raw.color.a = 1.0
        raw.lifetime = dense.lifetime
        for row in prediction["raw_map_xy"]:
            raw.points.append(Point(float(row[0]), float(row[1]), 0.25))
        markers.markers.append(raw)

        text = Marker()
        text.header = dense.header
        text.ns = dense.ns
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.orientation.w = 1.0
        text.pose.position.x = float(prediction["map_xy"][0, 0])
        text.pose.position.y = float(prediction["map_xy"][0, 1])
        text.pose.position.z = 1.5
        text.scale.z = 0.7
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = "TCP %s | target %.1f | wp %.1f / stop %.1f km/h" % (
            "STOP" if prediction["stop_active"] else "DRIVE",
            prediction["target_speed"] * 3.6,
            prediction["tcp_waypoint_speed"] * 3.6,
            prediction["stop_speed_threshold"] * 3.6,
        )
        text.lifetime = dense.lifetime
        markers.markers.append(text)
        return markers

    def publish_prediction(self, _event):
        with self.prediction_lock:
            prediction = self.latest_prediction
            prediction_time = self.latest_prediction_time
        if prediction is None:
            return
        if (
            prediction_time is None
            or (rospy.Time.now() - prediction_time).to_sec()
            > self.prediction_timeout
        ):
            self.target_speed_pub.publish(Float32(0.0))
            self.target_speed_kph_pub.publish(Float32(0.0))
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
        self.marker_pub.publish(self.make_markers(prediction))
        self.speed_profile_pub.publish(Float32MultiArray(
            data=np.full(len(map_path.poses), target, dtype=np.float32).tolist()
        ))
        self.target_speed_pub.publish(Float32(target))
        self.target_speed_kph_pub.publish(Float32(target * 3.6))
        self.tcp_speed_pub.publish(Float32(prediction["tcp_waypoint_speed"]))
        self.tcp_speed_head_pub.publish(Float32(prediction["tcp_speed_head"]))
        self.publish_tcp_mode(prediction["stop_active"])


if __name__ == "__main__":
    try:
        TCPInferenceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
