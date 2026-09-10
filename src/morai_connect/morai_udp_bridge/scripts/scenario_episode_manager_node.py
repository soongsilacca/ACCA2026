#!/usr/bin/env python3
"""Generate, load, and repeat MORAI teacher-driving episodes."""

import csv
import datetime
import glob
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rospy
from morai_msgs.msg import CollisionData, CtrlCmd, ObjectStatusList
from nav_msgs.msg import Odometry
from std_msgs.msg import String

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lib.define.ScenarioLoad import SetScenarioLoad
from lib.network.UDP import Sender
from domain_randomizer import generate_randomized_scenario
from ctypes import Structure, c_int, c_short, c_float, c_byte, c_char

class MultiEgoSettingData(Structure):
    _pack_ = 1
    _fields_ = [
        ("ego_index", c_short),
        ("position_x", c_float),
        ("position_y", c_float),
        ("position_z", c_float),
        ("roll", c_float),
        ("pitch", c_float),
        ("yaw", c_float),
        ("velocity", c_float),
        ("gear", c_byte),
        ("ctrl_mode", c_byte),
    ]

class MultiEgoSetting(Structure):
    _pack_ = 1
    _fields_ = [
        ("header", c_char * 17),
        ("data_length", c_int),
        ("aux_data", c_int * 3),
        ("Num_of_Ego", c_int),
        ("Cam_index", c_int),
        ("_data", MultiEgoSettingData * 20),
        ("tail", c_char * 2),
    ]


DEFAULT_RECORD_TOPICS = [
    "/camera/front/image/compressed",
    "/teacher/ground_truth_state",
    "/localization/kinematic_state",
    "/morai/ego_vehicle_status",
    "/global_path", "/local_route", "/mgeo_tokens",
    "/mgeo_target_velocity", "/privileged_expert/path",
    "/privileged_expert/target_velocity", "/privileged_expert/behavior",
    "/privileged_expert/mode_scores", "/privileged_expert/ready",
    "/Object_topic", "/SetTrafficLight",
    "/privileged_expert/controlled_traffic_light",
    "/ctrl_cmd", "/tracking_info", "/morai/collision_data",
    "/imu", "/gps", "/tf", "/tf_static",
]


class ScenarioEpisodeManager:
    def __init__(self):
        rospy.init_node("morai_scenario_episode_manager")
        self.input_scene = os.path.expanduser(rospy.get_param("~input_scene"))
        self.output_scene = os.path.expanduser(rospy.get_param("~output_scene"))
        self.active_output_scene = None
        self.global_path_csv = os.path.expanduser(rospy.get_param("~global_path_csv"))
        self.network_setting_file = os.path.expanduser(
            rospy.get_param("~network_setting_file", ""))
        self.load_network = bool(rospy.get_param("~load_network", True))
        # ScenarioLoad is itself delivered through MORAI's UDP network layer.
        # Reloading that layer in the same command can disconnect the receiver
        # before (or while) the scenario is applied.  Keep the complete profile
        # installed, but preserve the already-connected network during normal
        # episode resets.
        self.scenario_load_network = bool(
            rospy.get_param("~scenario_load_network", False))
        self.auto_connect_network = bool(
            rospy.get_param("~auto_connect_network", True))
        if self.load_network and not os.path.isfile(self.network_setting_file):
            raise RuntimeError("MORAI network setting file not found: %s" %
                               self.network_setting_file)
        self.network_entry_count = self._validate_network_profile(
            self.network_setting_file) if self.load_network else 0
        self.sender = Sender(rospy.get_param("~ip", "127.0.0.1"),
                             int(rospy.get_param("~port", 9095)))
        self.depart_radius = float(rospy.get_param("~departure_radius_m", 30.0))
        self.finish_radius = float(rospy.get_param("~finish_radius_m", 5.0))
        self.minimum_episode_sec = float(rospy.get_param("~minimum_episode_sec", 30.0))
        self.reload_settle_sec = float(rospy.get_param("~reload_settle_sec", 5.0))
        self.reload_ack_radius = float(
            rospy.get_param("~reload_ack_radius_m", 5.0))
        self.reload_retry_sec = float(
            rospy.get_param("~reload_retry_sec", 3.0))
        self.reload_max_attempts = int(
            rospy.get_param("~reload_max_attempts", 5))
        self.reload_on_collision = bool(rospy.get_param("~reload_on_collision", True))
        self.collision_actor_distance = float(
            rospy.get_param("~collision_actor_distance_m", 10.0))
        self.spawn_collision_ignore_radius = float(
            rospy.get_param("~spawn_collision_ignore_radius_m", 5.0))
        self.episode_timeout_sec = float(
            rospy.get_param("~episode_timeout_sec", 900.0))
        self.startup_load_delay_sec = float(
            rospy.get_param("~startup_load_delay_sec", 5.0))
        self.route_xy = self._read_route(self.global_path_csv)
        self.start_xy = self.route_xy[0]
        self.finish_index_margin = int(
            rospy.get_param("~finish_index_margin", 10))
        self.finish_stop_speed_mps = float(
            rospy.get_param("~finish_stop_speed_mps", 0.3))
        self.finish_stop_hold_sec = float(
            rospy.get_param("~finish_stop_hold_sec", 1.0))
        self.auto_record = bool(rospy.get_param("~auto_record", True))
        self.bag_directory = Path(os.path.expanduser(
            rospy.get_param("~bag_directory", "~/acca_ws/tcp_teacher_bags")))
        self.record_topics = list(rospy.get_param(
            "~record_topics", DEFAULT_RECORD_TOPICS))
        self.bag_split_size_mb = int(
            rospy.get_param("~bag_split_size_mb", 4096))
        self.bag_buffer_mb = int(rospy.get_param("~bag_buffer_mb", 2048))
        self.minimum_free_gb = float(rospy.get_param("~minimum_free_gb", 5.0))
        self.impact_fallback_enabled = bool(
            rospy.get_param("~impact_fallback_enabled", True))
        self.impact_pre_speed = float(
            rospy.get_param("~impact_pre_speed_mps", 1.5))
        self.impact_stop_speed = float(
            rospy.get_param("~impact_stop_speed_mps", 0.25))
        self.impact_drop_window = float(
            rospy.get_param("~impact_drop_window_sec", 0.8))
        self.impact_stop_hold = float(
            rospy.get_param("~impact_stop_hold_sec", 0.8))
        self.simulator_watchdog_enabled = bool(
            rospy.get_param("~simulator_watchdog_enabled", False))
        # The simulator watchdog publishes a latched READY/DOWN state.  When
        # enabled, never record or send ScenarioLoad while MORAI is absent.
        self.simulator_available = not self.simulator_watchdog_enabled
        self.startup_delay_elapsed = False
        self.departed = False
        self.collision_active = False
        self.loading = False
        self.transition_pending = False
        self.episode = 0
        self.episode_start = rospy.Time.now()
        self.last_pose = None
        self.route_index = 0
        self.finish_stop_since = None
        self.collision_armed = False
        self.drive_command_active = False
        self.last_fast_motion = rospy.Time(0)
        self.impact_stop_since = None
        self.reload_ack_scene = None
        self.reload_ack_attempts = 0
        self.reload_candidate_episode = 0
        self.reload_sent_wall = 0.0
        self.reload_gt_after_send = False
        self.reload_objects_after_send = False
        self.reload_expected_new_actors = {}
        self.last_object_positions = {}
        self.spawn_points_xy = np.empty((0, 2), dtype=float)
        self.object_state_received = False
        self.recorder = None
        self.bag_prefix = None
        self.transition_lock = threading.RLock()
        self.status_pub = rospy.Publisher("/morai/episode_status", String,
                                          queue_size=1, latch=True)
        self.recovery_pub = rospy.Publisher(
            "/morai/simulator_recovery_request", String, queue_size=1)
        rospy.Subscriber("/teacher/ground_truth_state", Odometry,
                         self._odom, queue_size=1)
        rospy.Subscriber("/morai/collision_data", CollisionData,
                         self._collision, queue_size=1)
        rospy.Subscriber("/ctrl_cmd", CtrlCmd, self._ctrl_cmd, queue_size=1)
        rospy.Subscriber("/Object_topic", ObjectStatusList,
                         self._objects, queue_size=1)
        rospy.Subscriber("/morai/simulator_status", String,
                         self._simulator_status, queue_size=1)
        rospy.Subscriber("/morai/episode_restart_request", String,
                         self._episode_restart_request, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self._watchdog)
        rospy.on_shutdown(self._shutdown)
        rospy.Timer(rospy.Duration(self.startup_load_delay_sec),
                    self._startup, oneshot=True)

    @staticmethod
    def _read_route(path):
        with open(path, newline="") as stream:
            rows = list(csv.DictReader(stream))
        if len(rows) < 2:
            raise RuntimeError("Global path must contain at least two points: %s" % path)
        return np.asarray([[float(row["x"]), float(row["y"])]
                           for row in rows], dtype=float)

    @staticmethod
    def _validate_network_profile(path):
        """Validate the complete MORAI profile without selecting UDP ports."""
        with open(path, encoding="utf-8") as stream:
            profile = json.load(stream)
        ego_data = profile.get("egoNetworkData")
        if not isinstance(ego_data, list) or not ego_data:
            raise RuntimeError("Invalid MORAI network profile (egoNetworkData): %s" % path)
        count = sum(len(ego.get("listEgoNetworkInfo", []))
                    for ego in ego_data if isinstance(ego, dict))
        if count == 0:
            raise RuntimeError("MORAI network profile has no network entries: %s" % path)
        rospy.loginfo("Using complete MORAI network profile: %s (%d entries)",
                      path, count)
        return count

    def _packet(self, scenario_name):
        data = SetScenarioLoad()
        data.filename = Path(scenario_name).stem.ljust(30).encode("utf-8")[:30]
        data.delete_all = False
        # Do not normally reload the network in the command that arrives over
        # that same network.  The generated <scene>_MN.json remains available
        # for bootstrap/recovery, while recurrent resets preserve Connect state.
        data.network = self.scenario_load_network
        data.ego = True
        data.npc = True
        data.pedestrian = True
        data.object = True
        data.pause = False
        return data

    @staticmethod
    def _expected_new_actor_positions(scene_path):
        """Return positions of actors appended by the randomizer.

        Base-scene IDs are below 1000; generated actors deliberately start at
        1000. Matching one of these actors in ObjectInfo proves MORAI parsed
        the newly written scenario, unlike a mere stream-liveness check.
        """
        with open(scene_path, encoding="utf-8") as stream:
            scenario = json.load(stream)
        expected = {}
        for list_name in ("pedestrianList", "objectList"):
            for actor in scenario.get(list_name, []):
                try:
                    uid = int(actor.get("UNIQUEID", -1))
                    pos = actor.get("pos", {})
                    if uid >= 1000:
                        expected[uid] = np.asarray(
                            [float(pos["x"]), float(pos["y"])])
                except (KeyError, TypeError, ValueError):
                    continue
        return expected

    @staticmethod
    def _spawn_point_positions(scene_path):
        """Read NPC spawn locations used to reject spawn-overlap events."""
        with open(scene_path, encoding="utf-8") as stream:
            scenario = json.load(stream)
        points = []
        for spawn in scenario.get("spawnPointList", []):
            try:
                pos = spawn["pos"]
                points.append([float(pos["x"]), float(pos["y"])])
            except (KeyError, TypeError, ValueError):
                continue
        return (np.asarray(points, dtype=float).reshape((-1, 2))
                if points else np.empty((0, 2), dtype=float))

    def _install_scenario_network_profile(self, scene_path):
        """Install the complete profile under MORAI's scenario-name convention.

        MORAI associates ``<scene>.json`` with
        ``EgoNetwork/<scene>_MN.json`` when ScenarioLoad.network is enabled.
        """
        if not self.load_network:
            return None
        scene = Path(scene_path)
        target_dir = scene.parent / "EgoNetwork"
        target_dir.mkdir(parents=True, exist_ok=True)
        target = target_dir / (scene.stem + "_MN.json")
        # Never mutate the user-owned source profile. MORAI persists the UI's
        # Connect state as connectedType in the scenario-specific _MN file, so
        # enable it only in this generated copy.
        with open(self.network_setting_file, encoding="utf-8") as stream:
            profile = json.load(stream)
        if self.auto_connect_network:
            for ego in profile.get("egoNetworkData", []):
                if ego.get("listEgoNetworkInfo"):
                    ego["connectedType"] = 1
        with open(str(target), "w", encoding="utf-8") as stream:
            json.dump(profile, stream, indent=2)
        return target

    @staticmethod
    def _cleanup_old_scenarios(base_dir, prefix="randomized_ep", keep_last=5):
        try:
            scene_files = sorted(base_dir.glob(f"{prefix}*.json"))
            if len(scene_files) > keep_last:
                for old_file in scene_files[:-keep_last]:
                    try:
                        old_file.unlink(missing_ok=True)
                    except Exception:
                        pass
                    mn_file = base_dir / "EgoNetwork" / f"{old_file.stem}_MN.json"
                    try:
                        mn_file.unlink(missing_ok=True)
                    except Exception:
                        pass
        except Exception:
            pass

    def _startup(self, _event):
        self.startup_delay_elapsed = True
        if (self.simulator_available and self.last_pose is not None and
                self.object_state_received):
            self._reload("startup")
        else:
            rospy.logwarn("Waiting for MORAI READY + fresh GT/ObjectInfo before ScenarioLoad")
            rospy.Timer(rospy.Duration(1.0), self._startup, oneshot=True)

    def _simulator_status(self, msg):
        state = msg.data.strip().upper()
        if state.startswith("DOWN"):
            if not self.simulator_available:
                return
            self.simulator_available = False
            self.last_pose = None
            self.object_state_received = False
            with self.transition_lock:
                self.loading = True
                if self.episode > 0 or self.recorder is not None:
                    rospy.logerr("MORAI exited; discarding current episode bag")
                    self._stop_recording(keep=False)
                self.status_pub.publish(String("SIMULATOR DOWN; EPISODE PAUSED"))
            return
        if not state.startswith("READY") or self.simulator_available:
            return
        self.simulator_available = True
        self.loading = False
        # Preserve the configured bootstrap delay on the very first run so
        # ScenarioLoad's UDP subscriber has time to connect.  READY after an
        # actual crash may resume immediately.
        if self.episode == 0 and not self.startup_delay_elapsed:
            return
        rospy.loginfo("MORAI simulator is ready; starting a fresh episode")
        self._reload("simulator_restart" if self.episode else "startup")

    def _episode_restart_request(self, msg):
        reason = msg.data.strip() or "external_request"
        if self.episode == 0 or self.loading or self.transition_pending:
            rospy.logwarn_throttle(
                2.0, "Ignoring episode restart request while initializing: %s",
                reason)
            return
        rospy.logerr("Episode restart requested: %s", reason)
        self._finish_episode(reason, success=False)

    def _bag_files(self):
        if self.bag_prefix is None:
            return []
        return [Path(path) for path in glob.glob(str(self.bag_prefix) + "*.bag*")]

    def _disk_has_room(self):
        self.bag_directory.mkdir(parents=True, exist_ok=True)
        free = shutil.disk_usage(str(self.bag_directory)).free
        return free >= int(self.minimum_free_gb * 1024 ** 3)

    def _start_recording(self):
        if not self.auto_record:
            return
        if not self._disk_has_room():
            rospy.logfatal("Bag recording stopped: less than %.1f GiB free in %s",
                           self.minimum_free_gb, self.bag_directory)
            rospy.signal_shutdown("bag storage full")
            return
        # Make each dataset self-describing: local recording start time and
        # the monotonically increasing lap number are visible in the name.
        # rosbag appends its own split-file suffix to this common prefix.
        stamp = datetime.datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        self.bag_prefix = self.bag_directory / (
            "teacher_%s_lap%06d" % (stamp, self.episode))
        command = ["rosbag", "record", "--lz4",
                   "--buffsize=%d" % self.bag_buffer_mb,
                   "--split", "--size=%d" % self.bag_split_size_mb,
                   "--output-name", str(self.bag_prefix)] + self.record_topics
        self.recorder = subprocess.Popen(command, start_new_session=True)
        rospy.loginfo("EPISODE %d RECORD START: %s*.bag",
                      self.episode, self.bag_prefix)

    def _stop_recording(self, keep):
        process = self.recorder
        self.recorder = None
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5.0)
            except ProcessLookupError:
                pass
        files = self._bag_files()
        if keep:
            rospy.loginfo("EPISODE %d RECORD SAVED: %d file(s)",
                          self.episode, len(files))
        else:
            removed = 0
            for path in files:
                try:
                    path.unlink()
                    removed += 1
                except FileNotFoundError:
                    pass
            rospy.logwarn("EPISODE %d RECORD DISCARDED: %d file(s)",
                          self.episode, removed)
        self.bag_prefix = None

    def _finish_episode(self, reason, success):
        with self.transition_lock:
            if self.loading or self.transition_pending:
                return
            self.transition_pending = True
            self.loading = True
            rospy.loginfo("EPISODE %d %s: %s", self.episode,
                          "SUCCESS" if success else "FAILED", reason)
            self._stop_recording(keep=success)
            self.loading = False
            if not rospy.is_shutdown() and self.simulator_available:
                self._reload(reason)

    def _reload(self, reason):
        with self.transition_lock:
            if not self.simulator_available:
                rospy.logwarn("Scenario reload deferred: MORAI simulator is down")
                return
            if self.loading:
                return
            if self.last_pose is None or not self.object_state_received:
                rospy.logwarn("Scenario reload deferred until GT/ObjectInfo network is ready")
                rospy.Timer(rospy.Duration(1.0),
                            lambda _event: self._reload(reason), oneshot=True)
                return
            self.loading = True
            self.transition_pending = True
            try:
                next_episode = self.episode + 1
                # MORAI indexes scenario names when the simulator starts.
                # Rewrite the pre-registered stable file rather than inventing
                # a runtime filename that may not exist in that index.
                active_scene = Path(self.output_scene)
                generate_randomized_scenario(
                    self.input_scene, str(active_scene), self.global_path_csv)
                self.spawn_points_xy = self._spawn_point_positions(
                    str(active_scene))
                rospy.loginfo("Loaded %d NPC spawn points; collisions within "
                              "%.1fm will be ignored",
                              len(self.spawn_points_xy),
                              self.spawn_collision_ignore_radius)
                network_target = self._install_scenario_network_profile(
                    str(active_scene))
                if network_target is not None:
                    rospy.loginfo("Installed complete scenario network profile: %s",
                                  network_target)
                
                self.reload_candidate_episode = next_episode
                self.reload_ack_scene = str(active_scene)
                self.reload_ack_attempts = 1
                self.reload_sent_wall = time.monotonic()
                self.reload_gt_after_send = False
                self.reload_objects_after_send = False
                self.reload_expected_new_actors = \
                    self._expected_new_actor_positions(str(active_scene))
                if not self.reload_expected_new_actors:
                    raise RuntimeError(
                        "Randomized scenario contains no generated actors for load ACK")
                self.active_output_scene = str(active_scene)
                self.sender.send(self._packet(str(active_scene)))
                self.collision_armed = False
                message = "EPISODE %d LOAD_SENT reason=%s scenario=%s" % (
                    next_episode, reason, active_scene.stem)
                self.status_pub.publish(String(message))
                rospy.loginfo(message)
                # Verify every load, including startup and a closed-loop route
                # whose goal is colocated with its start.
                rospy.Timer(rospy.Duration(self.reload_retry_sec),
                            self._verify_reload_ack, oneshot=True)
            except Exception as exc:
                rospy.logerr("Episode reload failed: %s", exc)
                self.reload_ack_scene = None
                self.loading = False
                self.recovery_pub.publish(String("scenario_reload_exception"))
            finally:
                rospy.Timer(rospy.Duration(self.reload_settle_sec),
                            self._release_loading, oneshot=True)

    def _maybe_complete_reload(self):
        if (self.reload_ack_scene is None or
                not self.reload_gt_after_send or
                not self.reload_objects_after_send):
            return False
        matched = False
        for uid, expected_xy in self.reload_expected_new_actors.items():
            actual_xy = self.last_object_positions.get(uid)
            if (actual_xy is not None and
                    np.linalg.norm(actual_xy - expected_xy) <= 1.5):
                matched = True
                break
        if not matched:
            return False

        self.episode = self.reload_candidate_episode
        self.reload_ack_scene = None
        self.transition_pending = False
        self.departed = False
        self.collision_active = False
        self.route_index = 0
        self.finish_stop_since = None
        self.last_fast_motion = rospy.Time(0)
        self.impact_stop_since = None
        self.episode_start = rospy.Time.now()
        message = "EPISODE %d LOADED scenario=%s" % (
            self.episode, Path(self.active_output_scene).stem)
        self.status_pub.publish(String(message))
        rospy.loginfo("Scenario reload ACK: GT/ObjectInfo match generated scene")
        rospy.loginfo(message)
        self._start_recording()
        return True

    def _release_loading(self, _event):
        if self.reload_ack_scene is not None:
            rospy.logwarn_throttle(
                3.0, "Keeping episode initialization locked until GT reload ACK")
            rospy.Timer(rospy.Duration(1.0),
                        self._release_loading, oneshot=True)
            return
        self.loading = False
        self.collision_armed = self.episode > 0
        rospy.loginfo("Episode collision detection ARMED")

    def _verify_reload_ack(self, _event):
        with self.transition_lock:
            if self.reload_ack_scene is None or rospy.is_shutdown():
                return
            if self._maybe_complete_reload():
                return
            if self.reload_ack_attempts >= self.reload_max_attempts:
                rospy.logerr("Scenario reload was not acknowledged after %d "
                             "UDP attempts; requesting simulator recovery",
                             self.reload_ack_attempts)
                self.reload_ack_scene = None
                self.status_pub.publish(String(
                    "EPISODE %d LOAD_FAILED scenario=%s" % (
                        self.reload_candidate_episode,
                        Path(self.active_output_scene).stem)))
                self.recovery_pub.publish(String("scenario_reload_no_ack"))
                return
            self.sender.send(self._packet(self.reload_ack_scene))
            self.reload_ack_attempts += 1
            # Every ScenarioLoad resend may make MORAI rebuild/drop its UDP
            # endpoints again. Notify the UI connector so it performs the
            # post-load settle and Connect verification for this attempt too.
            message = "EPISODE %d LOAD_SENT retry=%d/%d scenario=%s" % (
                self.reload_candidate_episode,
                self.reload_ack_attempts,
                self.reload_max_attempts,
                Path(self.reload_ack_scene).stem)
            self.status_pub.publish(String(message))
            rospy.logwarn("Scenario reload unacknowledged; resent %s (%d/%d)",
                          Path(self.reload_ack_scene).stem,
                          self.reload_ack_attempts,
                          self.reload_max_attempts)
            rospy.Timer(rospy.Duration(self.reload_retry_sec),
                        self._verify_reload_ack, oneshot=True)

    def _ctrl_cmd(self, msg):
        self.drive_command_active = (
            float(msg.brake) < 0.1 and
            ((int(msg.cmd_type) == 1 and float(msg.accel) > 0.1) or
             (int(msg.cmd_type) == 2 and float(msg.velocity) > 1.0)))

    def _odom(self, msg):
        xy = np.asarray([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.last_pose = xy
        if (self.reload_ack_scene is not None and
                time.monotonic() >= self.reload_sent_wall):
            self.reload_gt_after_send = True
            self._maybe_complete_reload()
        if self.loading:
            return
        # Track a monotonic path index in a forward window. This avoids a
        # closed route snapping from index 0 to the coincident final point.
        lo = max(0, self.route_index - 5)
        hi = min(len(self.route_xy), self.route_index + 400)
        local = int(np.argmin(np.linalg.norm(self.route_xy[lo:hi] - xy,
                                             axis=1)))
        self.route_index = max(self.route_index, lo + local)
        if self.route_index > self.finish_index_margin:
            self.departed = True
        speed = float(np.linalg.norm([
            msg.twist.twist.linear.x, msg.twist.twist.linear.y]))
        now = rospy.Time.now()

        if speed >= self.impact_pre_speed:
            self.last_fast_motion = now
            self.impact_stop_since = None
        elif (self.impact_fallback_enabled and self.drive_command_active and
              speed <= self.impact_stop_speed and
              self.last_fast_motion != rospy.Time(0) and
              (self.impact_stop_since is not None or
               (now - self.last_fast_motion).to_sec() <=
               self.impact_drop_window)):
            if self.impact_stop_since is None:
                self.impact_stop_since = now
            elif ((now - self.impact_stop_since).to_sec() >=
                  self.impact_stop_hold):
                rospy.logerr("Map-impact fallback: speed dropped from >=%.2f "
                             "to %.2f m/s while drive command remained active",
                             self.impact_pre_speed, speed)
                self.impact_stop_since = None
                self._finish_episode("map_impact_stall", success=False)
                return
        else:
            self.impact_stop_since = None
        at_end = self.route_index >= len(self.route_xy) - 1 - self.finish_index_margin
        if (self.departed and at_end and speed <= self.finish_stop_speed_mps and
                (now - self.episode_start).to_sec() >= self.minimum_episode_sec):
            if self.finish_stop_since is None:
                self.finish_stop_since = now
            elif (now - self.finish_stop_since).to_sec() >= self.finish_stop_hold_sec:
                self._finish_episode("route_complete", success=True)
        else:
            self.finish_stop_since = None

    def _objects(self, msg):
        positions = {}
        for items in (msg.npc_list, msg.pedestrian_list, msg.obstacle_list):
            for actor in items:
                positions[int(actor.unique_id)] = np.asarray(
                    [float(actor.position.x), float(actor.position.y)])
        self.last_object_positions = positions
        self.object_state_received = True
        if (self.reload_ack_scene is not None and
                time.monotonic() >= self.reload_sent_wall):
            self.reload_objects_after_send = True
            self._maybe_complete_reload()

    def _collision(self, msg):
        contacts = []
        spawn_contacts = []
        ego_involved = False
        for item in msg.collision_object:
            # MORAI includes this sentinel when the ego vehicle participates
            # in the reported collision.  Preserve that information instead
            # of merely dropping the entry; CollisionData also carries
            # unrelated NPC-vs-NPC collisions from elsewhere in the scene.
            if int(item.objType) == -1 and int(item.obj_id) == 0:
                ego_involved = True
                continue
            # Skip empty padding slots (all zeros)
            if (int(item.objType) == 0 and int(item.obj_id) == 0 and
                item.pose.x == 0.0 and item.pose.y == 0.0 and item.pose.z == 0.0 and
                item.globalOffset.x == 0.0 and item.globalOffset.y == 0.0 and item.globalOffset.z == 0.0):
                continue
            collision_xy = np.asarray(
                [float(item.pose.x), float(item.pose.y)])
            if len(self.spawn_points_xy):
                spawn_distance = float(np.min(np.linalg.norm(
                    self.spawn_points_xy - collision_xy, axis=1)))
                if spawn_distance <= self.spawn_collision_ignore_radius:
                    spawn_contacts.append((item, spawn_distance))
                    continue
            contacts.append(item)
        # This MORAI build also puts the ego sentinel in scene-wide NPC
        # collision packets. Validate the reported actor against ObjectInfo:
        # only an actor currently close to GT ego can invalidate the bag.
        ego_contacts = []
        remote_contacts = []
        for item in contacts:
            actor_xy = self.last_object_positions.get(int(item.obj_id))
            if actor_xy is None:
                actor_xy = np.asarray(
                    [float(item.pose.x), float(item.pose.y)])
            distance = (float(np.linalg.norm(actor_xy - self.last_pose))
                        if self.last_pose is not None else float('inf'))
            if distance <= self.collision_actor_distance:
                ego_contacts.append(item)
            else:
                remote_contacts.append((item, distance))
        contacts = ego_contacts
        collided = ego_involved and bool(contacts)
        self.collision_active = collided
        if spawn_contacts:
            rospy.logwarn_throttle(
                2.0, "Ignoring collision at NPC spawn point: %s",
                ", ".join("type=%d id=%d pos=(%.1f,%.1f) spawn_dist=%.2fm" %
                          (int(item.objType), int(item.obj_id),
                           float(item.pose.x), float(item.pose.y), distance)
                          for item, distance in spawn_contacts))
        if remote_contacts:
            rospy.logwarn_throttle(
                2.0, "Ignoring remote scene collision: %s",
                ", ".join("type=%d id=%d distance=%.1fm" %
                          (int(item.objType), int(item.obj_id),
                           distance)
                          for item, distance in remote_contacts))
        elif contacts and not ego_involved:
            rospy.logwarn_throttle(
                2.0, "Ignoring collision without ego sentinel")
        if not collided:
            return
        details = ", ".join("type=%d id=%d pos=(%.1f,%.1f)" %
                            (int(item.objType), int(item.obj_id), item.pose.x, item.pose.y)
                            for item in contacts)
        if self.loading or not self.collision_armed:
            rospy.logwarn_throttle(
                2.0, "Ignoring pre-arm/stale collision during scenario load: %s",
                details)
            return
        if (collided and self.collision_armed and self.reload_on_collision and
                not self.loading and
                (rospy.Time.now() - self.episode_start).to_sec() >= self.reload_settle_sec):
            rospy.logerr("Collision detected: %s", details)
            self._finish_episode("collision", success=False)

    def _watchdog(self, _event):
        if self.loading or self.transition_pending or self.episode == 0:
            return
        elapsed = (rospy.Time.now() - self.episode_start).to_sec()
        if elapsed >= self.episode_timeout_sec:
            self._finish_episode("timeout_%.0fs" % self.episode_timeout_sec,
                                 success=False)
            return
        if self.auto_record and self.recorder is not None:
            return_code = self.recorder.poll()
            if return_code is not None:
                rospy.logfatal("rosbag recorder exited unexpectedly (code=%d)",
                               return_code)
                self.recorder = None
                rospy.signal_shutdown("rosbag recorder failed or storage full")
            elif not self._disk_has_room():
                rospy.logfatal("Bag storage reached minimum free-space limit")
                rospy.signal_shutdown("bag storage full")

    def _shutdown(self):
        # A user Ctrl-C preserves the partial current episode; only explicitly
        # failed episodes (collision/timeout) are deleted.
        self._stop_recording(keep=True)


if __name__ == "__main__":
    ScenarioEpisodeManager()
    rospy.spin()
