#!/usr/bin/env python3
"""Model-agnostic closed-loop evaluation logger for MORAI runs."""

import csv
import datetime
import math
import os
import re
import threading
from pathlib import Path

import numpy as np
import rospy
from morai_msgs.msg import CollisionData, CtrlCmd, ObjectStatusList, TrackingInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from tf.transformations import euler_from_quaternion


TIME_FIELDS = [
    "timestamp", "model_name", "run_id", "scenario_id", "ego_x", "ego_y",
    "ego_yaw", "ego_speed", "steering_cmd", "longitudinal_cmd", "accel_cmd",
    "brake_cmd", "route_progress_m", "route_completion_ratio",
    "cross_track_error", "collision_flag", "collision_object",
    "traffic_light_state", "distance_to_stopline", "static_obstacle_flag",
    "dynamic_obstacle_flag", "distance_to_obstacle", "behavior", "inference_ms",
]

SUMMARY_FIELDS = [
    "model", "scenario", "seed", "run_id", "route_completion", "collision",
    "collision_object", "signal_success", "signal_events", "static_avoid_success",
    "static_events", "dynamic_stop_success", "dynamic_events", "max_cte",
    "mean_cte", "lane_departure", "mean_latency", "p95_latency",
    "duration_sec", "failure_reason",
]


class ClosedLoopEvaluator:
    def __init__(self):
        rospy.init_node("closed_loop_evaluator")
        self.model = str(rospy.get_param("~model_name", "unknown"))
        self.seed = int(rospy.get_param("~seed", -1))
        self.run_seed = self.seed
        self.output_dir = Path(os.path.expanduser(rospy.get_param(
            "~output_directory", "~/acca_ws/closed_loop_results")))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.route = self._read_route(os.path.expanduser(rospy.get_param(
            "~global_path_csv", "~/acca_ws/global_path/global_path.csv")))
        self.route_arc = np.r_[0.0, np.cumsum(np.linalg.norm(
            np.diff(self.route, axis=0), axis=1))]
        self.route_length = float(self.route_arc[-1])
        self.sample_hz = float(rospy.get_param("~sample_hz", 10.0))
        self.lane_departure_cte = float(rospy.get_param(
            "~lane_departure_cte_m", 2.0))
        self.obstacle_range = float(rospy.get_param(
            "~obstacle_event_range_m", 50.0))
        self.obstacle_path_width = float(rospy.get_param(
            "~obstacle_path_width_m", 4.0))
        self.signal_stop_speed = float(rospy.get_param(
            "~signal_stop_speed_mps", 0.3))
        self.signal_restart_speed = float(rospy.get_param(
            "~signal_restart_speed_mps", 1.0))
        self.signal_range = float(rospy.get_param(
            "~signal_event_range_m", 80.0))

        self.lock = threading.RLock()
        self.odom = None
        self.ctrl = None
        self.tracking = None
        self.objects = None
        self.behavior = "UNKNOWN"
        self.latency = float("nan")
        self.traffic_state = 0
        self.traffic_distance = float("nan")
        self.traffic_light = ""
        self.collision_flag = False
        self.collision_object = ""
        self.route_index = 0
        self.run_id = ""
        self.scenario = ""
        self.start_time = None
        self.stream = None
        self.writer = None
        self.ctes = []
        self.latencies = []
        self.max_completion = 0.0
        self.lane_departure = False
        self.static_ids = set()
        self.dynamic_ids = set()
        self.static_handled_ids = set()
        self.dynamic_handled_ids = set()
        self.static_failed_ids = set()
        self.dynamic_failed_ids = set()
        self.signal_events = 0
        self.signal_successes = 0
        self.signal_active = None
        self.signal_closed_lights = set()

        rospy.Subscriber(rospy.get_param(
            "~odom_topic", "/localization/kinematic_state"), Odometry,
                         self._odom, queue_size=1)
        rospy.Subscriber("/ctrl_cmd", CtrlCmd, self._ctrl, queue_size=1)
        rospy.Subscriber("/tracking_info", TrackingInfo,
                         self._tracking, queue_size=1)
        rospy.Subscriber("/Object_topic", ObjectStatusList,
                         self._objects, queue_size=1)
        rospy.Subscriber("/morai/collision_data", CollisionData,
                         self._collision, queue_size=1)
        rospy.Subscriber("/morai/episode_status", String,
                         self._episode_status, queue_size=10)
        rospy.Subscriber("/privileged_expert/controlled_traffic_light", String,
                         self._traffic, queue_size=1)
        rospy.Subscriber(rospy.get_param(
            "~behavior_topic", "/multimodal_learning/runtime_action"),
            String, self._behavior, queue_size=1)
        rospy.Subscriber(rospy.get_param(
            "~latency_topic", "/multimodal_learning/inference_latency_ms"),
            Float32, self._latency, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(self.sample_hz, 0.1)), self._sample)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo("Closed-loop evaluator ready: model=%s route=%.1fm output=%s",
                      self.model, self.route_length, self.output_dir)

    @staticmethod
    def _read_route(path):
        points = []
        with open(path, newline="") as stream:
            for row in csv.DictReader(stream):
                points.append((float(row["x"]), float(row["y"])))
        if len(points) < 2:
            raise RuntimeError("Evaluation route is empty: %s" % path)
        return np.asarray(points, dtype=float)

    def _reset_metrics(self):
        self.route_index = 0
        self.collision_flag = False
        self.collision_object = ""
        self.ctes = []
        self.latencies = []
        self.max_completion = 0.0
        self.lane_departure = False
        self.static_ids = set()
        self.dynamic_ids = set()
        self.static_handled_ids = set()
        self.dynamic_handled_ids = set()
        self.static_failed_ids = set()
        self.dynamic_failed_ids = set()
        self.signal_events = 0
        self.signal_successes = 0
        self.signal_active = None
        self.signal_closed_lights = set()

    def _start_run(self, episode, scenario, seed=None):
        self._finish_run("INTERRUPTED", write_summary=self.stream is not None)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_id = "%s_%s_ep%06d" % (self.model, stamp, episode)
        self.scenario = scenario
        self.run_seed = self.seed if seed is None else int(seed)
        self.start_time = rospy.Time.now()
        self._reset_metrics()
        run_dir = self.output_dir / self.run_id
        run_dir.mkdir(parents=True, exist_ok=True)
        self.stream = open(str(run_dir / "timeseries.csv"), "w", newline="")
        self.writer = csv.DictWriter(self.stream, fieldnames=TIME_FIELDS)
        self.writer.writeheader()
        rospy.loginfo("EVAL RUN START: %s scenario=%s", self.run_id, scenario)

    @staticmethod
    def _failure_reason(reason, collision_object=""):
        text = reason.lower()
        if text == "route_complete": return "SUCCESS"
        if "collision" in text:
            if "pedestrian" in text or collision_object == "pedestrian":
                return "COLLISION_PEDESTRIAN"
            if "vehicle" in text or collision_object == "vehicle":
                return "COLLISION_VEHICLE"
            return "COLLISION_STATIC"
        if "timeout" in text: return "TIMEOUT"
        if "localization" in text: return "LOCALIZATION_FAILURE"
        if "inference" in text: return "INFERENCE_FAILURE"
        if "stall" in text or "stuck" in text: return "STUCK"
        return reason.upper() if reason else "UNKNOWN"

    def _finish_run(self, reason, write_summary=True):
        if self.stream is None:
            return
        now = rospy.Time.now()
        duration = max(0.0, (now - self.start_time).to_sec())
        self.stream.flush()
        self.stream.close()
        self.stream = None
        self.writer = None
        if not write_summary:
            return
        ctes = np.asarray(self.ctes, dtype=float)
        latency = np.asarray(self.latencies, dtype=float)
        signal_rate = (float(self.signal_successes) / self.signal_events
                       if self.signal_events else float("nan"))
        if "collision" in reason.lower():
            self.collision_flag = True
            if not self.collision_object:
                if "pedestrian" in reason.lower():
                    self.collision_object = "pedestrian"
                elif "vehicle" in reason.lower():
                    self.collision_object = "vehicle"
                else:
                    self.collision_object = "static"
        static_ok = self.static_handled_ids - self.static_failed_ids
        dynamic_ok = self.dynamic_handled_ids - self.dynamic_failed_ids
        static_success = (float(len(static_ok)) /
                          len(self.static_ids) if self.static_ids else float("nan"))
        dynamic_success = (float(len(dynamic_ok)) /
                           len(self.dynamic_ids) if self.dynamic_ids else float("nan"))
        failure_reason = self._failure_reason(reason, self.collision_object)
        if failure_reason == "SUCCESS":
            if self.signal_events and self.signal_successes < self.signal_events:
                failure_reason = "SIGNAL_VIOLATION"
            elif self.lane_departure:
                failure_reason = "LANE_DEPARTURE"
        row = {
            "model": self.model, "scenario": self.scenario, "seed": self.run_seed,
            "run_id": self.run_id, "route_completion": self.max_completion,
            "collision": int(self.collision_flag),
            "collision_object": self.collision_object,
            "signal_success": signal_rate, "signal_events": self.signal_events,
            "static_avoid_success": static_success,
            "static_events": len(self.static_ids),
            "dynamic_stop_success": dynamic_success,
            "dynamic_events": len(self.dynamic_ids),
            "max_cte": float(np.max(np.abs(ctes))) if len(ctes) else float("nan"),
            "mean_cte": float(np.mean(np.abs(ctes))) if len(ctes) else float("nan"),
            "lane_departure": int(self.lane_departure),
            "mean_latency": float(np.mean(latency)) if len(latency) else float("nan"),
            "p95_latency": float(np.percentile(latency, 95)) if len(latency) else float("nan"),
            "duration_sec": duration,
            "failure_reason": failure_reason,
        }
        summary = self.output_dir / "summary.csv"
        new_file = not summary.exists()
        with open(str(summary), "a", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=SUMMARY_FIELDS)
            if new_file: writer.writeheader()
            writer.writerow(row)
        rospy.loginfo("EVAL RUN END: %s completion=%.1f%% reason=%s",
                      self.run_id, 100.0*self.max_completion,
                      row["failure_reason"])

    def _episode_status(self, msg):
        with self.lock:
            loaded = re.search(
                r"EPISODE\s+(\d+)\s+LOADED\s+scenario=(\S+)(?:\s+seed=(-?\d+))?",
                msg.data)
            if loaded:
                self._start_run(int(loaded.group(1)), loaded.group(2),
                                int(loaded.group(3)) if loaded.group(3) else None)
                return
            sent = re.search(r"LOAD_SENT\s+reason=([^\s]+)", msg.data)
            if sent and self.stream is not None:
                reason = sent.group(1)
                if reason not in ("startup", "simulator_restart"):
                    self._finish_run(reason)

    def _odom(self, msg): self.odom = msg
    def _ctrl(self, msg): self.ctrl = msg
    def _tracking(self, msg): self.tracking = msg
    def _objects(self, msg): self.objects = msg
    def _behavior(self, msg): self.behavior = msg.data
    def _latency(self, msg): self.latency = float(msg.data)

    def _route_maneuver(self):
        """Estimate the current intersection movement from route geometry."""
        i0 = min(self.route_index, len(self.route)-2)
        i1 = min(int(np.searchsorted(
            self.route_arc, self.route_arc[i0]+5.0)), len(self.route)-1)
        i2 = min(int(np.searchsorted(
            self.route_arc, self.route_arc[i0]+30.0)), len(self.route)-1)
        before = self.route[i1]-self.route[i0]
        after = self.route[i2]-self.route[i1]
        if np.linalg.norm(before) < 1e-6 or np.linalg.norm(after) < 1e-6:
            return "straight"
        turn = math.atan2(before[0]*after[1]-before[1]*after[0],
                          float(before.dot(after)))
        if turn < math.radians(-20.0): return "right"
        if turn > math.radians(20.0): return "left"
        return "straight"

    def _traffic(self, msg):
        match = re.search(
            r"^(\S+).*status=(\d+)\s+distance=([-\d.]+)m(?:\s+stopline=([-\d.]+)m)?",
            msg.data)
        if not match: return
        light, status = match.group(1), int(match.group(2))
        signal_distance = float(match.group(3))
        distance = (float(match.group(4)) if match.group(4) is not None
                    else signal_distance)
        self.traffic_light, self.traffic_state, self.traffic_distance = light, status, distance
        movement = self._route_maneuver()
        # MORAI masks: 1=red, 4=yellow, 16=straight green,
        # 33=red+protected-left. Right turns are unsignalized in this mission.
        if movement == "right":
            if (self.signal_active is not None and
                    self.signal_active["light"] == light):
                self.signal_events = max(0, self.signal_events-1)
            self.signal_active = None
            return
        red = (status in (1, 4) or
               (movement == "left" and status != 33) or
               (movement == "straight" and status != 16))
        if red and distance <= self.signal_range:
            if light in self.signal_closed_lights:
                return
            if self.signal_active is None or self.signal_active["light"] != light:
                if distance < -1.0:
                    return
                self.signal_events += 1
                self.signal_active = {"light": light, "stopped": False, "red": True}
            self.signal_active["red"] = True
            if distance < -1.0 and not self.signal_active["stopped"]:
                # The event remains in the denominator as a failed stop.
                self.signal_closed_lights.add(light)
                self.signal_active = None
        elif self.signal_active is not None and self.signal_active["light"] == light:
            # Success requires both a stop under red and a restart after release.
            self.signal_active["red"] = False

    @staticmethod
    def _valid_contacts(msg):
        return [item for item in msg.collision_object
                if not (int(item.objType) == -1 and int(item.obj_id) == 0)
                and not (int(item.objType) == 0 and int(item.obj_id) == 0 and
                         item.pose.x == 0.0 and item.pose.y == 0.0)]

    def _collision(self, msg):
        contacts = self._valid_contacts(msg)
        if not contacts or self.odom is None: return
        ego = np.asarray([self.odom.pose.pose.position.x,
                          self.odom.pose.pose.position.y])
        contact = min(contacts, key=lambda item: np.linalg.norm(
            np.asarray([item.pose.x, item.pose.y])-ego))
        if np.linalg.norm(np.asarray([contact.pose.x, contact.pose.y])-ego) > 10.0:
            return
        kind = "static"
        if self.objects is not None:
            uid = int(contact.obj_id)
            if any(int(obj.unique_id) == uid for obj in self.objects.pedestrian_list):
                kind = "pedestrian"
            elif any(int(obj.unique_id) == uid for obj in self.objects.npc_list):
                kind = "vehicle"
        self.collision_flag, self.collision_object = True, kind
        uid = int(contact.obj_id)
        if kind == "static": self.static_failed_ids.add(uid)
        else: self.dynamic_failed_ids.add(uid)

    def _route_state(self, xy):
        lo = max(0, self.route_index-20)
        hi = min(len(self.route), self.route_index+1000)
        local = np.linalg.norm(self.route[lo:hi]-xy, axis=1)
        idx = lo + int(np.argmin(local))
        self.route_index = max(self.route_index, idx)
        cte = float(np.min(local))
        completion = min(max(self.route_arc[self.route_index] /
                             max(self.route_length, 1e-6), 0.0), 1.0)
        return cte, completion

    def _obstacle_state(self, xy):
        static_flag = dynamic_flag = False
        closest = float("nan")
        static_near = set(); dynamic_near = set()
        if self.objects is None:
            return static_flag, dynamic_flag, closest, static_near, dynamic_near
        candidates = []
        for dynamic, group in ((False, self.objects.obstacle_list),
                               (True, self.objects.npc_list),
                               (True, self.objects.pedestrian_list)):
            for obj in group:
                p = np.asarray([obj.position.x, obj.position.y], dtype=float)
                distance = float(np.linalg.norm(p-xy))
                if distance > self.obstacle_range: continue
                lo, hi = self.route_index, min(len(self.route), self.route_index+1000)
                lateral = float(np.min(np.linalg.norm(self.route[lo:hi]-p, axis=1)))
                if lateral > self.obstacle_path_width + 0.5*float(obj.size.y): continue
                uid = int(obj.unique_id)
                candidates.append((distance, dynamic, uid))
                if dynamic:
                    dynamic_flag = True; self.dynamic_ids.add(uid); dynamic_near.add(uid)
                else:
                    static_flag = True; self.static_ids.add(uid); static_near.add(uid)
        if candidates: closest = min(item[0] for item in candidates)
        return static_flag, dynamic_flag, closest, static_near, dynamic_near

    def _sample(self, _event):
        with self.lock:
            if self.writer is None or self.odom is None: return
            msg = self.odom
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            cte, completion = self._route_state(np.asarray([p.x, p.y]))
            self.ctes.append(cte); self.max_completion = max(self.max_completion, completion)
            self.lane_departure |= abs(cte) > self.lane_departure_cte
            if math.isfinite(self.latency): self.latencies.append(self.latency)
            (static_flag, dynamic_flag, obstacle_distance,
             static_near, dynamic_near) = self._obstacle_state(
                 np.asarray([p.x, p.y]))
            if static_near and "AVOID" in self.behavior.upper():
                self.static_handled_ids.update(static_near)
            if dynamic_near and speed <= self.signal_stop_speed:
                self.dynamic_handled_ids.update(dynamic_near)
            if self.signal_active is not None:
                if self.signal_active["red"] and speed <= self.signal_stop_speed:
                    self.signal_active["stopped"] = True
                elif (not self.signal_active["red"] and
                      self.signal_active["stopped"] and speed >= self.signal_restart_speed):
                    self.signal_successes += 1
                    self.signal_closed_lights.add(self.signal_active["light"])
                    self.signal_active = None
            steer = float(self.ctrl.steer) if self.ctrl is not None else float("nan")
            accel = float(self.ctrl.accel) if self.ctrl is not None else float("nan")
            brake = float(self.ctrl.brake) if self.ctrl is not None else float("nan")
            longitudinal = accel-brake if self.ctrl is not None else float("nan")
            row = {
                "timestamp": rospy.Time.now().to_sec(), "model_name": self.model,
                "run_id": self.run_id, "scenario_id": self.scenario,
                "ego_x": p.x, "ego_y": p.y, "ego_yaw": yaw, "ego_speed": speed,
                "steering_cmd": steer, "longitudinal_cmd": longitudinal,
                "accel_cmd": accel, "brake_cmd": brake,
                "route_progress_m": completion*self.route_length,
                "route_completion_ratio": completion,
                "cross_track_error": cte,
                "collision_flag": int(self.collision_flag),
                "collision_object": self.collision_object,
                "traffic_light_state": self.traffic_state,
                "distance_to_stopline": self.traffic_distance,
                "static_obstacle_flag": int(static_flag),
                "dynamic_obstacle_flag": int(dynamic_flag),
                "distance_to_obstacle": obstacle_distance,
                "behavior": self.behavior, "inference_ms": self.latency,
            }
            self.writer.writerow(row)
            if len(self.ctes) % max(1, int(self.sample_hz*5.0)) == 0:
                self.stream.flush()

    def _shutdown(self):
        with self.lock:
            self._finish_run("INTERRUPTED", write_summary=self.stream is not None)


if __name__ == "__main__":
    ClosedLoopEvaluator()
    rospy.spin()
