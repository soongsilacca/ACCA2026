#!/usr/bin/env python3
"""ROS1 closed-loop adapter for the frozen MORAI TransFuser checkpoint."""

import math
import os
import threading
import time
import traceback

import numpy as np
import rospy
import torch
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import Float32, Float32MultiArray, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from multimodal_learning.lidar_bev import pointcloud2_to_xyz, points_to_bev
from multimodal_learning.sensor_preprocess import image_from_msg
from multimodal_learning.transfuser_morai import TransFuserConfig, TransFuserTrajectory


class TransFuserInferenceNode:
    def __init__(self):
        rospy.init_node("multimodal_inference", anonymous=False)
        default_checkpoint = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "model_artifacts",
            "transfuser_morai", "best.pt"))
        self.checkpoint_path = os.path.abspath(rospy.get_param(
            "~checkpoint_path", default_checkpoint))
        if not torch.cuda.is_available():
            raise RuntimeError("TransFuser inference requires CUDA")
        payload = torch.load(self.checkpoint_path, map_location="cpu", weights_only=False)
        if payload.get("schema") != "transfuser_morai_trajectory_v1":
            raise ValueError("checkpoint is not transfuser_morai_trajectory_v1")
        self.device = torch.device("cuda")
        self.dtype = torch.float16
        self.model = TransFuserTrajectory(
            TransFuserConfig(**payload.get("model_config", {})))
        self.model.load_state_dict(payload["model_state"], strict=True)
        self.model.to(device=self.device, dtype=self.dtype).eval()

        self.inference_period = float(rospy.get_param("~model_inference_period", 0.3))
        self.prediction_timeout = float(rospy.get_param("~prediction_timeout", 1.5))
        self.stop_speed_mps = float(rospy.get_param("~stop_speed_mps", 0.5))
        self.mgeo_speed_topic = rospy.get_param(
            "~mgeo_target_velocity_topic", "/mgeo_target_velocity")
        self.lock = threading.Lock()
        self.latest = {"left": None, "right": None, "lidar": None,
                       "odom": None, "status": None, "route": None,
                       "mgeo_speed": None}
        self.prediction = None
        self.prediction_wall_time = 0.0
        self.last_inference_stamp = float("-inf")

        self.path_pub = rospy.Publisher(
            "/multimodal_learning/predicted_path", Path, queue_size=1)
        self.local_path_pub = rospy.Publisher(
            "/multimodal_learning/predicted_path_local", Path, queue_size=1)
        self.target_speed_pub = rospy.Publisher("/target_velocity", Float32, queue_size=1)
        self.target_speed_kph_pub = rospy.Publisher(
            "/target_velocity_kph", Float32, queue_size=1)
        self.speed_profile_pub = rospy.Publisher(
            "/multimodal_learning/target_speeds", Float32MultiArray, queue_size=1)
        self.mode_pub = rospy.Publisher(
            "/multimodal_learning/mode_scores", Float32MultiArray, queue_size=1)
        self.action_pub = rospy.Publisher(
            "/multimodal_learning/runtime_action", String, queue_size=1)
        self.latency_pub = rospy.Publisher(
            "/multimodal_learning/inference_latency_ms", Float32, queue_size=1)

        rospy.Subscriber("/camera/left/image/compressed", CompressedImage,
                         lambda msg: self.set_latest("left", msg), queue_size=1,
                         buff_size=2 ** 24)
        rospy.Subscriber("/camera/right/image/compressed", CompressedImage,
                         lambda msg: self.set_latest("right", msg), queue_size=1,
                         buff_size=2 ** 24)
        rospy.Subscriber("/velodyne_points", PointCloud2,
                         lambda msg: self.set_latest("lidar", msg), queue_size=1,
                         buff_size=2 ** 24)
        rospy.Subscriber("/localization/kinematic_state", Odometry,
                         lambda msg: self.set_latest("odom", msg), queue_size=1)
        rospy.Subscriber("/morai/ego_vehicle_status", EgoVehicleStatus,
                         lambda msg: self.set_latest("status", msg), queue_size=1)
        rospy.Subscriber("/local_route", Path,
                         lambda msg: self.set_latest("route", msg), queue_size=1)
        rospy.Subscriber(self.mgeo_speed_topic, Float32,
                         lambda msg: self.set_latest("mgeo_speed", max(0.0, msg.data)),
                         queue_size=1)
        rospy.Subscriber("/camera/front/image/compressed", CompressedImage,
                         self.front_callback, queue_size=1, buff_size=2 ** 24)
        rospy.Timer(rospy.Duration(0.05), self.publish_prediction)
        rospy.loginfo(
            "TransFuser MORAI loaded on CUDA: %s epoch %s, period %.2fs",
            self.checkpoint_path, payload.get("epoch", -1), self.inference_period)

    def set_latest(self, key, value):
        with self.lock:
            self.latest[key] = value

    @staticmethod
    def route_in_base_link(route, odom):
        points = np.asarray([[pose.pose.position.x, pose.pose.position.y]
                             for pose in route.poses], np.float32)
        if route.header.frame_id.lstrip("/") in ("base_link", "base_footprint"):
            return points
        q = odom.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        delta = points - np.asarray(
            [odom.pose.pose.position.x, odom.pose.pose.position.y], np.float32)
        c, s = math.cos(yaw), math.sin(yaw)
        return np.column_stack((c * delta[:, 0] + s * delta[:, 1],
                                -s * delta[:, 0] + c * delta[:, 1]))

    @staticmethod
    def point_at_distance(points, distance):
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        stations = np.concatenate(([0.0], np.cumsum(lengths)))
        keep = np.r_[True, np.diff(stations) > 1.0e-4]
        stations, points = stations[keep], points[keep]
        distance = min(distance, float(stations[-1]))
        return np.asarray([np.interp(distance, stations, points[:, 0]),
                           np.interp(distance, stations, points[:, 1])], np.float32)

    @staticmethod
    def densify(points, spacing=0.2):
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        stations = np.concatenate(([0.0], np.cumsum(lengths)))
        keep = np.r_[True, np.diff(stations) > 1.0e-4]
        stations, points = stations[keep], points[keep]
        if len(points) < 2:
            return points
        samples = np.arange(0.0, stations[-1] + spacing * 0.5, spacing)
        return np.column_stack((np.interp(samples, stations, points[:, 0]),
                                np.interp(samples, stations, points[:, 1]))).astype(np.float32)

    def front_callback(self, front_msg):
        stamp = front_msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
        if stamp - self.last_inference_stamp < self.inference_period:
            return
        with self.lock:
            state = dict(self.latest)
        required = ("left", "right", "lidar", "odom", "route")
        if any(state[key] is None for key in required) or len(state["route"].poses) < 2:
            rospy.logwarn_throttle(2.0, "TransFuser waiting for cameras, LiDAR, odom and route")
            return
        try:
            left = image_from_msg(state["left"], 176, 160)
            front = image_from_msg(front_msg, 352, 160)
            right = image_from_msg(state["right"], 176, 160)
            panorama = np.concatenate((left, front, right), axis=2).astype(np.float32) / 255.0
            xyz = pointcloud2_to_xyz(state["lidar"])
            lidar = points_to_bev(xyz, (-10.0, 50.0), (-30.0, 30.0), 256, 256)
            route_xy = self.route_in_base_link(state["route"], state["odom"])
            target = self.point_at_distance(route_xy, 10.0)
            status = state["status"]
            if status is None:
                speed = abs(float(state["odom"].twist.twist.linear.x))
            else:
                speed = math.hypot(float(status.velocity.x),
                                   float(status.velocity.y)) / 3.6
            image_tensor = torch.from_numpy(panorama[None]).to(
                self.device, dtype=self.dtype, non_blocking=True)
            lidar_tensor = torch.from_numpy(lidar[None]).to(
                self.device, dtype=self.dtype, non_blocking=True)
            speed_tensor = torch.tensor([[speed]], device=self.device, dtype=self.dtype)
            target_tensor = torch.from_numpy(target[None]).to(
                self.device, dtype=self.dtype, non_blocking=True)
            started = time.perf_counter()
            with torch.inference_mode(), torch.autocast("cuda", dtype=self.dtype):
                output = self.model(image_tensor, lidar_tensor, speed_tensor, target_tensor)
            latency = (time.perf_counter() - started) * 1000.0
            self.latency_pub.publish(Float32(latency))
            waypoints = output["waypoints"][0].float().cpu().numpy()
            local_xy = self.densify(np.vstack((np.zeros((1, 2), np.float32), waypoints)))
            if len(local_xy) < 2 or not np.isfinite(local_xy).all():
                raise ValueError("invalid TransFuser trajectory")
            horizon_dt = np.asarray([0.6, 0.4, 0.6, 0.4], np.float32)
            segment_speed = np.linalg.norm(np.diff(
                np.vstack((np.zeros((1, 2), np.float32), waypoints)), axis=0), axis=1) / horizon_dt
            predicted_speed = float(np.clip(np.median(segment_speed), 0.0, 20.0))
            stop = predicted_speed <= self.stop_speed_mps
            target_speed = 0.0 if stop else predicted_speed
            if state["mgeo_speed"] is not None:
                target_speed = min(target_speed, float(state["mgeo_speed"]))
            odom = state["odom"]
            q = odom.pose.pose.orientation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            c, s = math.cos(yaw), math.sin(yaw)
            map_xy = np.column_stack((
                odom.pose.pose.position.x + c * local_xy[:, 0] - s * local_xy[:, 1],
                odom.pose.pose.position.y + s * local_xy[:, 0] + c * local_xy[:, 1]))
            self.prediction = dict(stamp=rospy.Time.from_sec(stamp), local=local_xy,
                                   map=map_xy, yaw=yaw, speed=target_speed,
                                   predicted_speed=predicted_speed, stop=stop)
            self.prediction_wall_time = time.monotonic()
            self.last_inference_stamp = stamp
            rospy.loginfo_throttle(
                1.0, "TransFuser %.0fms path=%d speed=%.1fkm/h %s",
                latency, len(local_xy), predicted_speed * 3.6,
                "STOP" if stop else "DRIVE")
        except Exception:
            rospy.logerr_throttle(2.0, "TransFuser inference failed:\n%s", traceback.format_exc())

    @staticmethod
    def make_path(points, frame, stamp, yaw_offset=0.0):
        message = Path()
        message.header.stamp = stamp
        message.header.frame_id = frame
        derivative = np.gradient(points, axis=0)
        headings = np.arctan2(derivative[:, 1], derivative[:, 0]) + yaw_offset
        for point, heading in zip(points, headings):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y = map(float, point)
            quaternion = quaternion_from_euler(0.0, 0.0, float(heading))
            pose.pose.orientation.x, pose.pose.orientation.y = quaternion[:2]
            pose.pose.orientation.z, pose.pose.orientation.w = quaternion[2:]
            message.poses.append(pose)
        return message

    def publish_prediction(self, _event):
        if rospy.is_shutdown():
            return
        prediction = self.prediction
        if prediction is None or time.monotonic() - self.prediction_wall_time > self.prediction_timeout:
            return
        stamp = rospy.Time.now()
        self.path_pub.publish(self.make_path(prediction["map"], "map", stamp))
        self.local_path_pub.publish(self.make_path(
            prediction["local"], "base_link", stamp))
        speed = float(prediction["speed"])
        self.target_speed_pub.publish(Float32(speed))
        self.target_speed_kph_pub.publish(Float32(speed * 3.6))
        self.speed_profile_pub.publish(Float32MultiArray(
            data=[speed] * len(prediction["local"])))
        # Controller contract is [STOP, DRIVE], matching TCP and SimLingo.
        self.mode_pub.publish(Float32MultiArray(
            data=[1.0, 0.0] if prediction["stop"] else [0.0, 1.0]))
        self.action_pub.publish(String(
            "TRANSFUSER_STOP" if prediction["stop"] else "TRANSFUSER_DRIVE"))


if __name__ == "__main__":
    try:
        TransFuserInferenceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
