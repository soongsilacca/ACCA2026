#!/usr/bin/env python3
"""ROS1 MORAI bridge for the SimLingo-Base MORAI teacher checkpoint."""

import math
import os
import threading
import time
import traceback

import numpy as np
import rospy
import torch
from geometry_msgs.msg import Point, PoseStamped
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import ColorRGBA, Float32, Float32MultiArray, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

_PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

from multimodal_learning.sensor_preprocess import image_from_msg
from multimodal_learning.simlingo_base_morai import (
    SimLingoBaseMorai,
    preprocess_front_image,
)


class SimLingoInferenceNode:
    def __init__(self):
        rospy.init_node("multimodal_inference", anonymous=False)
        self.checkpoint_path = os.path.abspath(rospy.get_param(
            "~checkpoint_path",
            os.path.join(
                _PACKAGE_DIR, "model_artifacts", "simlingo_base_teacher_v1", "best.pt"
            ),
        ))
        if not os.path.isfile(self.checkpoint_path):
            raise FileNotFoundError(self.checkpoint_path)
        if not torch.cuda.is_available():
            raise RuntimeError("SimLingo inference requires CUDA")
        self.device = torch.device("cuda")
        self.dtype = torch.float16
        self.model, self.checkpoint_epoch = self.load_model()

        self.inference_period = float(rospy.get_param(
            "~model_inference_period", 0.5
        ))
        self.publish_hz = float(rospy.get_param("~planning_publish_hz", 20.0))
        self.prediction_timeout = float(rospy.get_param(
            "~prediction_timeout", 1.5
        ))
        self.stop_speed_mps = float(rospy.get_param("~stop_speed_mps", 0.5))
        self.enable_speed_stop = bool(rospy.get_param(
            "~enable_speed_stop", True
        ))
        self.mgeo_speed_topic = rospy.get_param(
            "~mgeo_target_velocity_topic", "/mgeo_target_velocity"
        )
        self.state_lock = threading.Lock()
        self.latest_odom = self.latest_status = self.latest_route = None
        self.latest_mgeo_speed = None
        self.last_inference_stamp = float("-inf")
        self.prediction_lock = threading.Lock()
        self.latest_prediction = self.latest_prediction_time = None

        self.path_pub = rospy.Publisher(
            "/multimodal_learning/predicted_path", Path, queue_size=1
        )
        self.local_path_pub = rospy.Publisher(
            "/multimodal_learning/predicted_path_local", Path, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            "/multimodal_learning/predicted_path_markers", MarkerArray,
            queue_size=1,
        )
        self.speed_profile_pub = rospy.Publisher(
            "/multimodal_learning/target_speeds", Float32MultiArray,
            queue_size=1,
        )
        self.target_speed_pub = rospy.Publisher(
            "/target_velocity", Float32, queue_size=1
        )
        self.target_speed_kph_pub = rospy.Publisher(
            "/target_velocity_kph", Float32, queue_size=1
        )
        self.mode_pub = rospy.Publisher(
            "/multimodal_learning/mode_scores", Float32MultiArray,
            queue_size=1,
        )
        self.runtime_action_pub = rospy.Publisher(
            "/multimodal_learning/runtime_action", String, queue_size=1
        )
        rospy.Subscriber("/localization/kinematic_state", Odometry,
                         self.odom_callback, queue_size=1)
        rospy.Subscriber("/morai/ego_vehicle_status", EgoVehicleStatus,
                         self.status_callback, queue_size=1)
        rospy.Subscriber("/local_route", Path, self.route_callback, queue_size=1)
        rospy.Subscriber(self.mgeo_speed_topic, Float32,
                         self.mgeo_speed_callback, queue_size=1)
        camera_topic = rospy.get_param(
            "~camera_topic", "/camera/front/image/compressed"
        )
        camera_type = CompressedImage if camera_topic.endswith("/compressed") else Image
        rospy.Subscriber(camera_topic, camera_type, self.camera_callback,
                         queue_size=1, buff_size=2 ** 24)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_hz, 1.0)),
            self.publish_prediction,
        )
        rospy.loginfo(
            "SimLingo-Base MORAI teacher loaded on CUDA: %s, epoch %d, period %.2fs",
            self.checkpoint_path, self.checkpoint_epoch, self.inference_period,
        )

    def load_model(self):
        checkpoint = torch.load(
            self.checkpoint_path, map_location="cpu", weights_only=False
        )
        if checkpoint.get("schema") != "simlingo_base_morai_v1":
            raise ValueError("checkpoint is not simlingo_base_morai_v1")
        model = SimLingoBaseMorai()
        model.load_state_dict(checkpoint["model_state"], strict=True)
        model.to(device=self.device, dtype=self.dtype).eval()
        return model, int(checkpoint.get("epoch", -1))

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
        with self.state_lock:
            self.latest_mgeo_speed = max(float(msg.data), 0.0)

    @staticmethod
    def route_in_base_link(route, odom):
        points = np.asarray([[p.pose.position.x, p.pose.position.y]
                             for p in route.poses], dtype=np.float32)
        frame = route.header.frame_id.lstrip("/")
        if frame in ("base_link", "base_footprint"):
            return points
        q = odom.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        delta = points - np.asarray([
            odom.pose.pose.position.x, odom.pose.pose.position.y
        ], dtype=np.float32)
        c, s = math.cos(yaw), math.sin(yaw)
        return np.column_stack((c * delta[:, 0] + s * delta[:, 1],
                                -s * delta[:, 0] + c * delta[:, 1]))

    @staticmethod
    def sample_route(route_xy, stations):
        segment = np.linalg.norm(np.diff(route_xy, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(segment)))
        keep = np.r_[True, np.diff(cumulative) > 1.0e-4]
        cumulative, route_xy = cumulative[keep], route_xy[keep]
        stations = np.minimum(np.asarray(stations), cumulative[-1])
        return np.column_stack((np.interp(stations, cumulative, route_xy[:, 0]),
                                np.interp(stations, cumulative, route_xy[:, 1]))).astype(np.float32)

    def camera_callback(self, msg):
        stamp_sec = msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
        if stamp_sec - self.last_inference_stamp < self.inference_period:
            return
        with self.state_lock:
            odom, status, route = self.latest_odom, self.latest_status, self.latest_route
            mgeo_speed = self.latest_mgeo_speed
        if odom is None or route is None or len(route.poses) < 2:
            rospy.logwarn_throttle(2.0, "SimLingo waiting for odometry and Local Route")
            return
        try:
            route_xy = self.route_in_base_link(route, odom)
            speed = abs(float(odom.twist.twist.linear.x)) if status is None else math.hypot(
                float(status.velocity.x), float(status.velocity.y)) / 3.6
            image = image_from_msg(msg, 672, 336)
            patches = preprocess_front_image(image)[None].to(
                self.device, dtype=self.dtype, non_blocking=True
            )
            target_point = torch.from_numpy(
                self.sample_route(route_xy, (10.0,))[0]
            )[None].to(self.device, dtype=self.dtype, non_blocking=True)
            speed_tensor = torch.tensor(
                [speed], device=self.device, dtype=self.dtype
            )
            started = time.perf_counter()
            with torch.inference_mode(), torch.autocast("cuda", dtype=self.dtype):
                geometric, temporal = self.model(patches, speed_tensor, target_point)
            geometric_xy = geometric[0].float().cpu().numpy()
            temporal_xy = temporal[0].float().cpu().numpy()
            # The driving output is the 2-second temporal trajectory.  Include
            # the ego origin so MPC receives a path beginning at the vehicle.
            local_xy = np.vstack((np.zeros((1, 2), np.float32), temporal_xy))
            latency = (time.perf_counter() - started) * 1000.0
            if not np.isfinite(local_xy).all() or len(local_xy) < 2:
                raise ValueError("non-finite/empty SimLingo path")
            steps = np.linalg.norm(np.diff(local_xy, axis=0), axis=1) / 0.2
            predicted_speed = float(np.median(steps[:5])) if len(steps) else 0.0
            predicted_speed = float(np.clip(predicted_speed, 0.0, 20.0))
            # This artifact has no STOP classifier. Preserve its continuous
            # temporal speed prediction as speed authority instead of
            # converting a small value into a discrete full-brake command.
            stop = (
                self.enable_speed_stop
                and predicted_speed <= self.stop_speed_mps
            )
            target_speed = 0.0 if stop else predicted_speed
            if mgeo_speed is not None:
                target_speed = min(target_speed, mgeo_speed)
            q = odom.pose.pose.orientation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            c, s = math.cos(yaw), math.sin(yaw)
            map_xy = np.column_stack((
                odom.pose.pose.position.x + c * local_xy[:, 0] - s * local_xy[:, 1],
                odom.pose.pose.position.y + s * local_xy[:, 0] + c * local_xy[:, 1],
            )).astype(np.float32)
            derivative = np.gradient(local_xy, axis=0)
            local_yaw = np.arctan2(derivative[:, 1], derivative[:, 0])
            prediction = dict(stamp=rospy.Time.from_sec(stamp_sec), local_xy=local_xy,
                              local_yaw=local_yaw, map_xy=map_xy,
                              map_yaw=local_yaw + yaw, target_speed=target_speed,
                              predicted_speed=predicted_speed, stop=stop, latency=latency,
                              geometric_xy=geometric_xy)
            with self.prediction_lock:
                self.latest_prediction = prediction
                self.latest_prediction_time = rospy.Time.now()
            self.last_inference_stamp = stamp_sec
            rospy.loginfo_throttle(1.0, "SimLingo %.0fms path=%d speed=%.1fkm/h %s",
                                   latency, len(local_xy), predicted_speed * 3.6,
                                   "STOP" if stop else "DRIVE")
        except Exception as error:
            rospy.logerr_throttle(
                2.0, "SimLingo inference failed:\n%s", traceback.format_exc()
            )

    @staticmethod
    def make_path(xy, yaw, stamp, frame):
        result = Path(); result.header.stamp = stamp; result.header.frame_id = frame
        for point, angle in zip(xy, yaw):
            pose = PoseStamped(); pose.header = result.header
            pose.pose.position.x, pose.pose.position.y = map(float, point)
            q = quaternion_from_euler(0.0, 0.0, float(angle))
            pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
            pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
            result.poses.append(pose)
        return result

    def make_markers(self, p):
        marker = Marker(); marker.header.frame_id = "map"; marker.header.stamp = p["stamp"]
        marker.ns = "simlingo_predicted_path"; marker.id = 0
        marker.type = Marker.LINE_STRIP; marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0; marker.scale.x = 0.35
        marker.color = ColorRGBA(1.0, 0.05, 0.65, 0.95); marker.lifetime = rospy.Duration(1.0)
        marker.points = [Point(float(x), float(y), 0.2) for x, y in p["map_xy"]]
        text = Marker(); text.header = marker.header; text.ns = marker.ns; text.id = 1
        text.type = Marker.TEXT_VIEW_FACING; text.action = Marker.ADD
        text.pose.orientation.w = 1.0; text.pose.position.x = float(p["map_xy"][0, 0])
        text.pose.position.y = float(p["map_xy"][0, 1]); text.pose.position.z = 1.5
        text.scale.z = 0.65; text.color = ColorRGBA(1, 1, 1, 1)
        text.text = "SimLingo %s | %.1f km/h | %.0f ms" % (
            "STOP" if p["stop"] else "DRIVE", p["predicted_speed"] * 3.6, p["latency"])
        text.lifetime = marker.lifetime
        return MarkerArray(markers=[marker, text])

    def publish_prediction(self, _event):
        with self.prediction_lock:
            p, updated = self.latest_prediction, self.latest_prediction_time
        if p is None or (rospy.Time.now() - updated).to_sec() > self.prediction_timeout:
            return
        map_path = self.make_path(p["map_xy"], p["map_yaw"], p["stamp"], "map")
        local_path = self.make_path(p["local_xy"], p["local_yaw"], p["stamp"], "base_link")
        self.path_pub.publish(map_path); self.local_path_pub.publish(local_path)
        self.marker_pub.publish(self.make_markers(p))
        self.speed_profile_pub.publish(Float32MultiArray(
            data=[p["target_speed"]] * len(map_path.poses)))
        self.target_speed_pub.publish(Float32(p["target_speed"]))
        self.target_speed_kph_pub.publish(Float32(p["target_speed"] * 3.6))
        self.mode_pub.publish(Float32MultiArray(
            data=[1.0, 0.0] if p["stop"] else [0.0, 1.0]))
        self.runtime_action_pub.publish(String("SIMLINGO_STOP" if p["stop"] else "SIMLINGO_DRIVE"))


if __name__ == "__main__":
    SimLingoInferenceNode()
    rospy.spin()
