#!/usr/bin/env python3
"""ROS/MORAI I/O adapter for the unmodified CARLA Garage privileged AutoPilot."""
import json
import math
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import rospy
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList, SetTrafficLight
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool, ColorRGBA, Float32, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from morai_carla_compat import (ActorList, Location, Provider, Rotation, TrafficLight,
                                TrafficLightState, Transform, Vector3D, Vehicle,
                                VehicleControl, Walker, WalkerControl, Waypoint, World)
from upstream_pdm_loader import AutoPilot, RoadOption
from privileged_route_planner import PrivilegedRoutePlanner
from longitudinal_controller import LongitudinalLinearRegressionController


class CommandPlannerAdapter:
    def __init__(self, route): self.route = route
    def run_step(self, position):
        if not self.route: return [(np.asarray(position[:2]), RoadOption.LANEFOLLOW)]
        xy = np.asarray([p[0] for p in self.route])
        idx = int(np.argmin(np.linalg.norm(xy[:, :2] - np.asarray(position[:2]), axis=1)))
        return self.route[idx:idx+3] or [self.route[-1]]


class MoraiUpstreamPDM:
    def __init__(self):
        rospy.init_node("morai_upstream_pdm")
        self.lock = threading.RLock()
        self.odom = self.ego = self.objects = self.global_path = None
        self.speed_limit = 13.89
        self.object_frame = rospy.get_param("~object_frame", "map")
        self.hz = float(rospy.get_param("~publish_hz", 10.0))
        self.horizon = float(rospy.get_param("~publish_horizon_m", 60.0))
        self.map_dir = rospy.get_param("~map_dir", "")
        self.traffic = {}
        self.traffic_group_anchor = None
        self.traffic_group_radius = float(rospy.get_param("~traffic_group_radius_m", 15.0))
        self.traffic_stop_buffer = float(rospy.get_param("~traffic_stop_buffer_m", 20.0))
        self.traffic_stop_buffer_overrides = {
            str(key): float(value) for key, value in
            rospy.get_param("~traffic_stop_buffer_overrides", {}).items()
        }
        self.traffic_camera_based_stop = bool(rospy.get_param("~traffic_camera_based_stop", False))
        self.traffic_camera_view_distance = float(rospy.get_param("~traffic_camera_view_distance_m", 30.0))
        self.traffic_camera_search_back = float(rospy.get_param("~traffic_camera_search_back_m", 60.0))
        self.traffic_path_tolerance = float(rospy.get_param("~traffic_path_tolerance_m", 6.0))
        self.traffic_link_match_tolerance = float(rospy.get_param("~traffic_link_match_tolerance_m", 4.0))
        self.traffic_lookahead = float(rospy.get_param("~traffic_lookahead_m", 180.0))
        self.traffic_commit_margin = float(rospy.get_param("~traffic_commit_margin_m", 3.0))
        self.traffic_clear_distance = float(rospy.get_param("~traffic_clear_distance_m", 12.0))
        self.target_speed_limit_ratio = float(rospy.get_param("~target_speed_limit_ratio", 1.0))
        self.pedestrian_forecast_length = float(rospy.get_param("~pedestrian_forecast_length_sec", 4.0))
        self.stationary_pedestrian_speed = float(rospy.get_param("~stationary_pedestrian_speed_mps", 0.2))
        self.stationary_pedestrian_confirm = float(rospy.get_param("~stationary_pedestrian_confirm_sec", 1.0))
        self.actor_detection_radius = float(rospy.get_param("~actor_detection_radius_m", 80.0))
        self.static_avoid_min_distance = float(rospy.get_param("~static_avoid_min_distance_m", 3.0))
        self.static_avoid_max_distance = float(rospy.get_param("~static_avoid_max_distance_m", 60.0))
        self.static_avoid_lateral_tolerance = float(rospy.get_param("~static_avoid_lateral_tolerance_m", 4.0))
        self.static_avoid_clearance = float(rospy.get_param("~static_avoid_clearance_m", 1.5))
        self.static_avoid_max_lane_factor = float(rospy.get_param("~static_avoid_max_lane_factor", 2.0))
        self.static_avoid_speed = float(rospy.get_param("~static_avoid_speed_mps", 5.0))
        self.pedestrian_hazard_full_stop = bool(rospy.get_param("~pedestrian_hazard_full_stop", True))
        self.pedestrian_caution_horizon = float(rospy.get_param("~pedestrian_caution_horizon_sec", 6.0))
        self.pedestrian_caution_clearance = float(rospy.get_param("~pedestrian_caution_clearance_m", 3.5))
        self.pedestrian_caution_decel = float(rospy.get_param("~pedestrian_caution_decel_mps2", 2.5))
        self.pedestrian_stop_margin = float(rospy.get_param("~pedestrian_stop_margin_m", 8.0))
        self.pedestrian_caution_max_speed = float(rospy.get_param("~pedestrian_caution_max_speed_mps", 8.0))
        self.pedestrian_front_slow_distance = float(rospy.get_param("~pedestrian_front_slow_distance_m", 40.0))
        self.pedestrian_front_slow_speed = float(rospy.get_param("~pedestrian_front_slow_speed_mps", 5.0))
        self.pedestrian_front_half_angle = math.radians(float(rospy.get_param("~pedestrian_front_half_angle_deg", 40.0)))
        self.npc_yaw_rate_ema = float(rospy.get_param("~npc_yaw_rate_ema", 0.5))
        self.npc_max_yaw_rate = math.radians(float(
            rospy.get_param("~npc_max_yaw_rate_deg_sec", 55.0)))
        self.npc_turn_min_speed = float(rospy.get_param(
            "~npc_turn_min_speed_mps", 0.5))
        self.goal_slowdown_distance = float(
            rospy.get_param("~goal_slowdown_distance_m", 20.0))
        self.goal_stop_distance = float(
            rospy.get_param("~goal_stop_distance_m", 1.5))
        self.goal_decel = float(rospy.get_param("~goal_decel_mps2", 2.0))
        self.goal_index_margin = int(rospy.get_param("~goal_index_margin", 3))
        self.signal_points, self.signal_routes, self.signal_visual_points = self._load_signal_map(self.map_dir)
        self.agent = None; self.route_signature = None
        self.route_bootstrap_recovered = False
        self.processed_static_ids = set()
        self.committed_light_ids = {}
        self.signal_diagnostic = None
        self.pedestrian_diagnostic = None
        self.pedestrian_tracks = {}
        self.vehicle_tracks = {}

        self.path_pub = rospy.Publisher("/privileged_expert/path", Path, queue_size=1)
        self.speed_pub = rospy.Publisher("/privileged_expert/target_velocity", Float32, queue_size=1)
        self.behavior_pub = rospy.Publisher("/privileged_expert/behavior", String, queue_size=1)
        self.mode_pub = rospy.Publisher("/privileged_expert/mode_scores", Float32MultiArray, queue_size=1)
        self.ready_pub = rospy.Publisher("/privileged_expert/ready", Bool, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("/privileged_expert/markers", MarkerArray, queue_size=1)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/localization/kinematic_state"),
                         Odometry, self._odom, queue_size=1)
        rospy.Subscriber("/morai/ego_vehicle_status", EgoVehicleStatus, self._ego, queue_size=1)
        rospy.Subscriber("/global_path", Path, self._path, queue_size=1)
        rospy.Subscriber("/mgeo_target_velocity", Float32, self._speed, queue_size=1)
        rospy.Subscriber(rospy.get_param("~object_topic", "/Object_topic"), ObjectStatusList,
                         self._objects, queue_size=1)
        rospy.Subscriber(rospy.get_param("~traffic_command_topic", "/SetTrafficLight"),
                         SetTrafficLight, self._traffic, queue_size=20)
        rospy.Subscriber("/morai/episode_status", String,
                         self._episode_status, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/max(self.hz, 1.0)), self._tick)
        rospy.loginfo("Official CARLA Garage AutoPilot MORAI adapter started (MPC path/speed output only)")

    @staticmethod
    def _load_signal_map(map_dir):
        node_path = os.path.join(map_dir, "node_set.json")
        link_path = os.path.join(map_dir, "link_set.json")
        if not os.path.isfile(node_path): return {}, {}, {}
        try:
            nodes = json.load(open(node_path, encoding="utf-8"))
            node_lights = {str(n["idx"]): str(n["traffic_light_id"])
                           for n in nodes if n.get("traffic_light_id")}
            points = {str(n["traffic_light_id"]): np.asarray(n["point"][:3], dtype=float)
                      for n in nodes if n.get("traffic_light_id") and n.get("point")}
            routes = {}
            if os.path.isfile(link_path):
                for link in json.load(open(link_path, encoding="utf-8")):
                    light_id = node_lights.get(str(link.get("from_node_idx")))
                    xy = np.asarray(link.get("points", []), dtype=float)
                    if light_id and len(xy) >= 2:
                        routes.setdefault(light_id, []).append(
                            (str(link.get("related_signal") or "").lower(), xy[:, :2]))
            visual_points = {}
            control_path = os.path.join(map_dir, "traffic_light_control_set.json")
            if os.path.isfile(control_path):
                for light in json.load(open(control_path, encoding="utf-8")):
                    if light.get("idx") and light.get("point"):
                        visual_points[str(light["idx"])] = np.asarray(light["point"][:3], dtype=float)
            return points, routes, visual_points
        except Exception as exc:
            rospy.logwarn("Traffic-light map unavailable: %s", exc); return {}, {}, {}

    def _camera_visible_stop_target(self, light_id, signal_route_index, route_begin):
        """Return route index/buffer that keeps the physical signal in camera view."""
        planner = self.agent._waypoint_planner
        visual = self.signal_visual_points.get(light_id) if self.traffic_camera_based_stop else None
        if visual is None:
            stop_buffer = self.traffic_stop_buffer_overrides.get(light_id, self.traffic_stop_buffer)
            fallback = int(stop_buffer * planner.points_per_meter)
            target_index = max(route_begin, signal_route_index-fallback)
            return target_index, (signal_route_index-target_index)/float(planner.points_per_meter)
        search = int(self.traffic_camera_search_back * planner.points_per_meter)
        first = max(route_begin, signal_route_index-search)
        candidates = planner.original_route_points[first:signal_route_index+1, :2]
        if len(candidates) == 0:
            return signal_route_index, 0.0
        visual_distances = np.linalg.norm(candidates-visual[:2], axis=1)
        local_index = int(np.argmin(np.abs(visual_distances-self.traffic_camera_view_distance)))
        target_index = first+local_index
        return target_index, (signal_route_index-target_index)/float(planner.points_per_meter)

    def _odom(self, msg):
        with self.lock: self.odom = msg
    def _ego(self, msg):
        with self.lock: self.ego = msg
    def _path(self, msg):
        with self.lock: self.global_path = msg
    def _speed(self, msg):
        with self.lock: self.speed_limit = max(0.0, float(msg.data))
    def _episode_status(self, msg):
        if "LOADED" not in msg.data:
            return
        with self.lock:
            self.agent = None
            self.route_signature = None
            self.route_bootstrap_recovered = False
            # ScenarioLoad teleports/recreates the ego asynchronously. Never
            # initialize the new PDM route from the last pose/object packet of
            # the previous episode; wait for fresh post-load UDP state.
            self.odom = None
            self.ego = None
            self.objects = None
            self.processed_static_ids.clear()
            self.committed_light_ids.clear()
            self.traffic.clear()
            self.traffic_group_anchor = None
            self.pedestrian_tracks.clear()
            self.vehicle_tracks.clear()
            rospy.loginfo("PDM reset for %s", msg.data)
    def _objects(self, msg):
        with self.lock:
            now = rospy.Time.now().to_sec()
            vehicle_seen = set()
            for vehicle in msg.npc_list:
                actor_id = int(vehicle.unique_id); vehicle_seen.add(actor_id)
                heading = math.radians(float(vehicle.heading))
                previous = self.vehicle_tracks.get(actor_id)
                if previous is None:
                    self.vehicle_tracks[actor_id] = dict(
                        heading=heading, stamp=now, yaw_rate=0.0)
                    continue
                dt = now - previous["stamp"]
                if dt > 1e-3:
                    delta = math.atan2(
                        math.sin(heading-previous["heading"]),
                        math.cos(heading-previous["heading"]))
                    measured = float(np.clip(
                        delta/dt, -self.npc_max_yaw_rate,
                        self.npc_max_yaw_rate))
                    alpha = float(np.clip(self.npc_yaw_rate_ema, 0.0, 1.0))
                    previous["yaw_rate"] = (
                        alpha*measured + (1.0-alpha)*previous["yaw_rate"])
                previous["heading"] = heading
                previous["stamp"] = now
            self.vehicle_tracks = {
                key: value for key, value in self.vehicle_tracks.items()
                if key in vehicle_seen or now-value["stamp"] < 1.0
            }
            seen = set()
            for pedestrian in msg.pedestrian_list:
                actor_id = int(pedestrian.unique_id); seen.add(actor_id)
                position = np.asarray([pedestrian.position.x, pedestrian.position.y], dtype=float)
                previous = self.pedestrian_tracks.get(actor_id)
                if previous is None:
                    self.pedestrian_tracks[actor_id] = dict(position=position, stamp=now,
                                                           first_stamp=now, speed=float("inf"))
                    continue
                dt = now-previous["stamp"]
                measured = float(np.linalg.norm(position-previous["position"])/dt) if dt > 1e-3 else previous["speed"]
                previous["speed"] = measured if not np.isfinite(previous["speed"]) else .5*previous["speed"]+.5*measured
                previous["position"] = position; previous["stamp"] = now
            self.pedestrian_tracks = {key: value for key, value in self.pedestrian_tracks.items()
                                      if key in seen or now-value["stamp"] < 1.0}
            self.objects = msg

    def _pedestrian_is_stationary(self, actor_id):
        track = self.pedestrian_tracks.get(int(actor_id))
        return bool(track is not None and
                    rospy.Time.now().to_sec()-track["first_stamp"] >= self.stationary_pedestrian_confirm and
                    track["speed"] < self.stationary_pedestrian_speed)

    def _npc_forecast_steer(self, actor_id, speed):
        """Convert measured NPC yaw rate to the upstream bicycle-model steer.

        MORAI ObjectInfo has no control command. Leaving steer at zero makes a
        vehicle circulating through a roundabout travel along its tangent for
        the full forecast and creates a false collision at the ego merge.
        """
        track = self.vehicle_tracks.get(int(actor_id))
        if track is None or speed < self.npc_turn_min_speed or self.agent is None:
            return 0.0
        model = self.agent.vehicle_model
        sine_slip = float(np.clip(
            track["yaw_rate"] * model.rear_wheel_base / max(speed, 1e-3),
            -0.95, 0.95))
        slip = math.asin(sine_slip)
        wheelbase = model.front_wheel_base + model.rear_wheel_base
        wheel_angle = math.atan(
            math.tan(slip) * wheelbase / model.rear_wheel_base)
        return float(np.clip(wheel_angle / model.steering_gain, -1.0, 1.0))
    def _traffic(self, msg):
        with self.lock:
            light_id = str(msg.trafficLightIndex)
            point = self.signal_points.get(light_id)
            # The traffic manager emits one packet per signal head. Preserve
            # every head belonging to the same intersection; clear the old
            # intersection only when the next packet is spatially separate.
            if point is not None:
                if (self.traffic_group_anchor is None or
                        np.linalg.norm(point[:2] - self.traffic_group_anchor[:2]) > self.traffic_group_radius):
                    self.traffic = {}
                    self.traffic_group_anchor = point.copy()
            elif light_id not in self.traffic:
                self.traffic = {}
                self.traffic_group_anchor = None
            self.traffic[light_id] = int(msg.trafficLightStatus)

    @staticmethod
    def _yaw(odom):
        q = odom.pose.pose.orientation
        return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    @staticmethod
    def _dense_path(path, ppm):
        raw = np.asarray([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in path.poses])
        if len(raw) < 2: return None
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(raw[:, :2], axis=0), axis=1))]
        keep = np.r_[True, np.diff(arc) > 1e-4]; raw, arc = raw[keep], arc[keep]
        if len(raw) < 2: return None
        stations = np.arange(0., arc[-1] + .5/ppm, 1./ppm)
        return np.column_stack([np.interp(stations, arc, raw[:, i]) for i in range(3)])

    def _initialize(self, path):
        agent = AutoPilot(); agent.setup("")
        # Upstream data collection intentionally drives at 72% of the map
        # limit. The MORAI teacher should follow the authored MGeo speed; only
        # hazards/IDM are allowed to reduce it.
        agent.config.ratio_target_speed_limit = self.target_speed_limit_ratio
        agent.config.default_forecast_length = self.pedestrian_forecast_length
        agent.config.forecast_length_lane_change = self.pedestrian_forecast_length
        agent.config.detection_radius = self.actor_detection_radius
        points = self._dense_path(path, agent.config.points_per_meter)
        if points is None: return False
        delta = np.gradient(points[:, :2], axis=0)
        yaws = np.degrees(np.arctan2(delta[:, 1], delta[:, 0]))
        wps = [Waypoint(Transform(Location(*p), Rotation(yaw=y))) for p, y in zip(points, yaws)]
        planner = PrivilegedRoutePlanner(agent.config)
        planner.route_points = points.copy(); planner.original_route_points = points.copy()
        planner.route_waypoints = wps
        planner.commands = [RoadOption.LANEFOLLOW] * len(points)
        planner.commands_orig = planner.commands.copy()
        planner.rotation_angles = yaws
        planner.distances_to_next_traffic_lights = np.full(len(points), np.inf)
        planner.next_traffic_lights = [None] * len(points)
        planner.distances_to_next_stop_signs = np.full(len(points), np.inf)
        planner.next_stop_signs = [None] * len(points)
        planner.speed_limits = np.full(len(points), max(self.speed_limit, .1))
        if self.odom is not None:
            ego_xy = np.asarray([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y])
            # This competition route is a closed lap, so its final point is
            # spatially adjacent to the start. A whole-route nearest search
            # can select that final point after ScenarioLoad and publish only
            # 0-1 remaining poses, causing MPC's safe stop. New episodes
            # always spawn at the authored route start; constrain bootstrap
            # matching to its first 100 m.
            bootstrap_count = min(
                len(points), max(2, int(100.0 * agent.config.points_per_meter)))
            initial_index = int(np.argmin(np.linalg.norm(
                points[:bootstrap_count, :2] - ego_xy, axis=1)))
        else:
            initial_index = 0
        planner.route_index = planner.last_route_index = initial_index

        world = World()
        world.map.waypoint_resolver = lambda loc: wps[int(np.argmin(
            np.linalg.norm(points[:, :2]-np.asarray([loc.x, loc.y]), axis=1)))]
        hero = Vehicle(1, "vehicle.morai.ego", extent=Vector3D(2.4, .95, .8))
        Provider.world = world; Provider.hero = hero; Provider.active_scenarios = []
        world.actors = ActorList([hero])
        agent._vehicle = hero; agent._world = world; agent.world_map = world.map
        agent._waypoint_planner = planner
        agent._longitudinal_controller = LongitudinalLinearRegressionController(agent.config)
        command_route = [(p.copy(), RoadOption.LANEFOLLOW) for p in points[::max(1, agent.config.points_per_meter)]]
        agent._command_planner = CommandPlannerAdapter(command_route)
        agent.list_traffic_lights = []
        agent.initialized = True
        self.agent = agent
        self.processed_static_ids.clear()
        self.committed_light_ids.clear()
        self.route_signature = (len(path.poses), round(points[-1, 0], 2), round(points[-1, 1], 2))
        rospy.loginfo("Upstream PDM route initialized: %d dense points", len(points))
        return True

    def _update_world(self, odom, ego, objects):
        agent = self.agent; hero = agent._vehicle
        p = odom.pose.pose.position; yaw = math.degrees(self._yaw(odom))
        hero._transform = Transform(Location(p.x, p.y, p.z), Rotation(yaw=yaw))
        v = odom.twist.twist.linear
        hero._velocity = Vector3D(v.x, v.y, v.z)
        if ego is not None and ego.size.x > 0:
            hero.bounding_box.extent = Vector3D(ego.size.x/2., ego.size.y/2., max(ego.size.z/2., .5))
        actors = ActorList([hero])
        static_actors = []
        if objects is not None:
            groups = ((objects.npc_list, Vehicle, "vehicle.morai.npc"),
                      (objects.pedestrian_list, Walker, "walker.pedestrian.morai"),
                      (objects.obstacle_list, Vehicle, "vehicle.morai.static"))
            for items, cls, type_id in groups:
                for item in items:
                    if cls is Walker and self._pedestrian_is_stationary(item.unique_id):
                        continue
                    pos = item.position; vel = item.velocity
                    actor_yaw = math.radians(float(item.heading))
                    # ObjectInfo velocity is expressed in the object's body
                    # frame. CARLA actors/WalkerControl require map-frame
                    # motion for future collision prediction.
                    world_vx = math.cos(actor_yaw)*vel.x - math.sin(actor_yaw)*vel.y
                    world_vy = math.sin(actor_yaw)*vel.x + math.cos(actor_yaw)*vel.y
                    # MORAI ObjectInfo carries absolute map altitude while the
                    # localization used by this stack is flattened to z=0.
                    # Keep every actor in the ego planning plane so upstream
                    # 3-D OBB intersection does not miss real XY collisions.
                    actor = cls(1000+int(item.unique_id), type_id,
                                Transform(Location(pos.x, pos.y, hero.get_location().z), Rotation(yaw=item.heading)),
                                Vector3D(world_vx, world_vy, vel.z),
                                Vector3D(max(item.size.x/2., .2), max(item.size.y/2., .2), max(item.size.z/2., .2)))
                    if isinstance(actor, Walker):
                        speed = actor._velocity.length()
                        actor._control = WalkerControl(
                            Vector3D(world_vx/speed, world_vy/speed, vel.z/speed)
                            if speed > .01 else Vector3D())
                    elif type_id.endswith("npc"):
                        # Feed measured turning motion into the unmodified
                        # upstream forecast. This matters at a roundabout
                        # entrance: a circulating car must follow its arc,
                        # rather than being projected straight across the ego
                        # merge path for four seconds.
                        actor._control = VehicleControl(
                            steer=self._npc_forecast_steer(
                                item.unique_id, actor._velocity.length()))
                    if type_id.endswith("static"):
                        # Static MORAI obstacles are handled by the upstream
                        # ParkedObstacle scenario, which shifts the route
                        # around the actor. Do not also expose the same object
                        # as a normal vehicle: that duplicates it in generic
                        # collision forecasting and incorrectly latches STOP
                        # before the shifted AVOID path can be followed.
                        static_actors.append(actor)
                    else:
                        actors.append(actor)
        agent._world.actors = actors
        # ScenarioRunner normally supplies these entries in CARLA. ObjectInfo
        # is the MORAI-side privileged equivalent; only the representation is
        # adapted, while upstream route-shift logic remains untouched.
        if not Provider.active_scenarios:
            planner = agent._waypoint_planner
            for actor in static_actors:
                if actor.id in self.processed_static_ids: continue
                delta = planner.original_route_points[planner.route_index:, :2] - np.asarray(
                    [actor.get_location().x, actor.get_location().y])
                local_idx = int(np.argmin(np.linalg.norm(delta, axis=1)))
                distance = local_idx / float(planner.points_per_meter)
                lateral = float(np.linalg.norm(delta[local_idx]))
                route_idx = planner.route_index + local_idx
                tangent = planner.original_route_points[min(route_idx+1, len(planner.route_points)-1), :2] - planner.original_route_points[max(route_idx-1, 0), :2]
                tangent_norm = float(np.linalg.norm(tangent))
                if tangent_norm < 1e-6:
                    continue
                tangent /= tangent_norm
                normal = np.asarray([-tangent[1], tangent[0]])
                actor_yaw = math.radians(actor.get_transform().rotation.yaw)
                actor_forward = np.asarray([math.cos(actor_yaw), math.sin(actor_yaw)])
                actor_normal = np.asarray([-actor_forward[1], actor_forward[0]])
                extent = actor.bounding_box.extent
                projected_half_length = (abs(float(actor_forward.dot(tangent))) * extent.x +
                                         abs(float(actor_normal.dot(tangent))) * extent.y)
                projected_half_width = (abs(float(actor_forward.dot(normal))) * extent.x +
                                        abs(float(actor_normal.dot(normal))) * extent.y)
                if (self.static_avoid_min_distance <= distance <= self.static_avoid_max_distance and
                        lateral <= self.static_avoid_lateral_tolerance + projected_half_width):
                    offset = np.asarray([actor.get_location().x, actor.get_location().y]) - planner.original_route_points[route_idx, :2]
                    side = tangent[0]*offset[1] - tangent[1]*offset[0]
                    direction = "left" if side > 0 else "right"
                    signed_lateral = float(offset.dot(normal))
                    required_shift = max(
                        0.0, projected_half_width + hero.bounding_box.extent.y +
                        self.static_avoid_clearance - abs(signed_lateral))
                    lane_width = max(float(planner.route_waypoints[route_idx].lane_width), 0.1)
                    lane_factor = min(self.static_avoid_max_lane_factor,
                                      max(1.0, required_shift / lane_width))

                    # Keep the upstream route-shift implementation, but pass
                    # MORAI bounding-box-aware geometry. The stock scenario
                    # always shifts exactly one lane and cannot clear wide or
                    # rotated obstacles.
                    original_half_length = extent.x
                    extent.x = max(extent.x, projected_half_length)
                    planner.shift_route_around_actors(
                        actor, actor, direction,
                        transition_length=agent.config.transition_smoothness_distance,
                        lane_transition_factor=lane_factor)
                    extent.x = original_half_length
                    rospy.loginfo("Static AVOID id=%d size=%.1fx%.1fm lateral=%.1fm shift=%.1fm (%.2f lanes)",
                                  actor.id, 2.0*projected_half_length,
                                  2.0*projected_half_width, signed_lateral,
                                  lane_width*lane_factor, lane_factor)
                    self.processed_static_ids.add(actor.id)
                    break

    def _update_lights(self):
        planner = self.agent._waypoint_planner; n = len(planner.route_points)
        planner.distances_to_next_traffic_lights[:] = np.inf
        planner.next_traffic_lights = [None] * n; lights = []
        begin = planner.route_index
        end = min(n, begin + int(self.traffic_lookahead * planner.points_per_meter))
        route_window = planner.original_route_points[begin:end, :2]
        if len(route_window) == 0:
            return
        diagnostics = []
        for light_id, status in self.traffic.items():
            point = self.signal_points.get(light_id)
            if point is None: continue
            separation = np.linalg.norm(route_window - point[:2], axis=1)
            local_idx = int(np.argmin(separation))
            if float(separation[local_idx]) > self.traffic_path_tolerance:
                continue
            idx = begin + local_idx

            # Geometry fallback for maps where related_signal is missing or
            # associated with a neighboring signal head. A clockwise route
            # turn is a right turn and does not obey the vehicle signal in
            # this mission.
            look = int(10.0 * planner.points_per_meter)
            before_idx = max(begin, idx - look)
            after_idx = min(n - 1, idx + look)
            incoming = planner.original_route_points[idx, :2] - planner.original_route_points[before_idx, :2]
            outgoing_geometry = planner.original_route_points[after_idx, :2] - planner.original_route_points[idx, :2]
            if np.linalg.norm(incoming) > 1e-6 and np.linalg.norm(outgoing_geometry) > 1e-6:
                geometry_turn = math.atan2(
                    incoming[0]*outgoing_geometry[1] - incoming[1]*outgoing_geometry[0],
                    float(np.dot(incoming, outgoing_geometry)))
                if geometry_turn < -math.radians(20.0):
                    continue

            # A signal head is relevant only if the route leaving its stop
            # line follows one of that signal's MGeo outgoing links. This
            # rejects cross-road and opposing-direction lights at the same
            # intersection while still retaining multiple heads for our lane.
            outgoing = self.signal_routes.get(light_id, [])
            matched_movement = ""
            if outgoing:
                sample_end = min(n, idx + int(30.0 * planner.points_per_meter))
                route_after = planner.original_route_points[idx:sample_end:planner.points_per_meter, :2]
                matched = False
                best_match = None
                if len(route_after) >= 2:
                    route_direction = route_after[-1] - route_after[0]
                    route_norm = np.linalg.norm(route_direction)
                    for movement, link_xy in outgoing:
                        link_direction = link_xy[-1] - link_xy[0]
                        denom = route_norm * np.linalg.norm(link_direction)
                        if denom <= 1e-6 or np.dot(route_direction, link_direction) / denom < 0.3:
                            continue
                        distances = np.linalg.norm(route_after[:, None, :] - link_xy[None, :, :], axis=2)
                        score = float(np.median(np.min(distances, axis=1)))
                        if best_match is None or score < best_match[0]:
                            best_match = (score, movement)
                    if best_match is not None and best_match[0] <= self.traffic_link_match_tolerance:
                        matched = True
                        matched_movement = best_match[1]
                if not matched:
                    continue
            # Right turns in this MORAI mission are unsignalized. Pedestrian
            # and vehicle collision prediction remains active.
            if matched_movement.startswith("right"):
                diagnostics.append(dict(light_id=light_id, status=status, movement="right",
                                        distance=(idx-begin)/float(planner.points_per_meter),
                                        permitted=True, ignored=True, route_index=idx))
                continue
            light_location = Location(point[0], point[1], self.agent._vehicle.get_location().z)
            light = TrafficLight(abs(hash(light_id)) % 1000000 + 2000000, "traffic.traffic_light",
                                 Transform(light_location))
            # MORAI bit 33 means RED + protected LEFT. It must not release a
            # straight route merely because bit 32 is present. Permission is
            # evaluated against the MGeo movement selected above.
            if matched_movement in ("left", "left_unprotected"):
                # Every left turn is protected in this MORAI teacher setup:
                # proceed only on RED + LEFT ARROW (status 33), never on the
                # ordinary straight-green phase.
                permitted = bool((status & 1) and (status & 32) and not (status & 16))
            elif matched_movement == "right":
                # Korean right turn: the vehicle signal itself does not hold
                # the route. Pedestrian/vehicle collision checks still do.
                permitted = True
            else:  # straight/unknown require the ordinary green phase
                permitted = bool(status & 16)

            raw_route_distance = (idx - begin) / float(planner.points_per_meter)
            target_idx, stop_buffer = self._camera_visible_stop_target(light_id, idx, begin)
            diagnostics.append(dict(light_id=light_id, status=status,
                                    movement=matched_movement or "straight",
                                    distance=raw_route_distance, permitted=permitted,
                                    ignored=(matched_movement == "right"), route_index=idx,
                                    stop_target_index=target_idx, stop_buffer=stop_buffer,
                                    camera_based=(self.traffic_camera_based_stop and
                                                  light_id in self.signal_visual_points)))
            if light_id in self.committed_light_ids:
                # The green entry decision was already made. Ignore a phase
                # change until the rear of the ego has cleared the junction.
                if begin <= self.committed_light_ids[light_id]:
                    continue
                self.committed_light_ids.pop(light_id, None)
            if permitted and raw_route_distance <= stop_buffer + self.traffic_commit_margin:
                # Commit the entire intersection group because its signal
                # heads arrive as sequential UDP messages.
                clear_index = idx + int(self.traffic_clear_distance * planner.points_per_meter)
                for group_light_id in self.traffic:
                    self.committed_light_ids[group_light_id] = clear_index
                continue
            light.state = (TrafficLightState.Green if permitted else
                           (TrafficLightState.Yellow if status & 4 else TrafficLightState.Red))
            lights.append(light)
            for i in range(idx + 1):
                if planner.next_traffic_lights[i] is None:
                    planner.next_traffic_lights[i] = light
                    # Present a virtual stop target before the mapped stop
                    # line. The unmodified upstream IDM then performs the
                    # actual deceleration against this shorter distance.
                    planner.distances_to_next_traffic_lights[i] = max(
                        0.0, (idx-i)/float(planner.points_per_meter) - stop_buffer)
        self.agent.list_traffic_lights = [(l, l.get_location(), [self.agent.world_map.get_waypoint(l.get_location())]) for l in lights]
        self.agent._world.actors.extend(lights)
        self.signal_diagnostic = min(diagnostics, key=lambda item: item["distance"]) if diagnostics else None

    def _pedestrian_speed_cap(self, objects, route, current_target):
        self.pedestrian_diagnostic = None
        if objects is None or len(route) < 2 or not objects.pedestrian_list:
            return current_target
        route_xy = np.asarray(route[:, :2], dtype=float)
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(route_xy, axis=0), axis=1))]
        cap = float(current_target)
        times = np.linspace(0.0, self.pedestrian_caution_horizon, 25)
        ego_position = np.asarray([self.odom.pose.pose.position.x,
                                   self.odom.pose.pose.position.y])
        ego_yaw = self._yaw(self.odom)
        forward = np.asarray([math.cos(ego_yaw), math.sin(ego_yaw)])
        for pedestrian in objects.pedestrian_list:
            stationary = self._pedestrian_is_stationary(pedestrian.unique_id)
            yaw = math.radians(float(pedestrian.heading))
            vx = math.cos(yaw)*pedestrian.velocity.x - math.sin(yaw)*pedestrian.velocity.y
            vy = math.sin(yaw)*pedestrian.velocity.x + math.cos(yaw)*pedestrian.velocity.y
            future = np.column_stack((pedestrian.position.x + times*vx,
                                      pedestrian.position.y + times*vy))
            # Slow down as soon as a pedestrian is in the forward field of
            # view. This is only a caution cap: full stop remains governed by
            # upstream predicted route collision, so a person on a distant
            # sidewalk cannot latch STOP.
            offset = future[0] - ego_position
            front_distance = float(np.linalg.norm(offset))
            forward_distance = float(np.dot(offset, forward))
            if 0.0 < front_distance <= self.pedestrian_front_slow_distance and forward_distance > 0.0:
                bearing = abs(math.atan2(forward[0]*offset[1]-forward[1]*offset[0],
                                         float(np.dot(forward, offset))))
                if bearing <= self.pedestrian_front_half_angle:
                    blend = front_distance / max(self.pedestrian_front_slow_distance, .1)
                    front_cap = self.pedestrian_front_slow_speed + blend * max(
                        float(current_target)-self.pedestrian_front_slow_speed, 0.0)
                    cap = min(cap, front_cap)
                    if (self.pedestrian_diagnostic is None or
                            (self.pedestrian_diagnostic.get("front_only", False) and
                             front_cap < self.pedestrian_diagnostic["cap"])):
                        self.pedestrian_diagnostic = dict(
                            actor_id=int(pedestrian.unique_id), clearance=float("inf"),
                            distance=front_distance, cap=front_cap, future=future,
                            conflict=future[0], front_only=True, stationary=stationary)
            # A stationary person ahead causes cautious driving but is not
            # inserted into route-conflict/full-stop prediction.
            if stationary:
                continue
            distances = np.linalg.norm(future[:, None, :] - route_xy[None, :, :], axis=2)
            future_idx, route_idx = np.unravel_index(int(np.argmin(distances)), distances.shape)
            if float(distances[future_idx, route_idx]) > self.pedestrian_caution_clearance:
                continue
            conflict_distance = float(arc[route_idx])
            if conflict_distance <= 0.0:
                continue
            braking_distance = max(conflict_distance - self.pedestrian_stop_margin, 0.0)
            approach_cap = math.sqrt(2.0*self.pedestrian_caution_decel*braking_distance)
            cap = min(cap, self.pedestrian_caution_max_speed, approach_cap)
            candidate = dict(actor_id=int(pedestrian.unique_id), clearance=float(distances[future_idx, route_idx]),
                             distance=conflict_distance, cap=approach_cap, future=future,
                             conflict=future[future_idx], front_only=False, stationary=False)
            if (self.pedestrian_diagnostic is None or self.pedestrian_diagnostic.get("front_only", False) or
                    candidate["distance"] < self.pedestrian_diagnostic["distance"]):
                self.pedestrian_diagnostic = candidate
        return max(cap, 0.0)

    @staticmethod
    def _speed_color(speed, maximum=13.89):
        ratio = min(max(float(speed) / max(maximum, .1), 0.0), 1.0)
        return ColorRGBA(0.55 + .45*ratio, 0.05, 0.85*(1.0-ratio), .82)

    def _decision_markers(self, points, speed, behavior, hard_stop, data):
        now = rospy.Time.now(); array = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; array.markers.append(clear)
        if len(points) < 2 or self.odom is None:
            return array

        # A continuous road-width ribbon is easier to read than discrete path points.
        ribbon = Marker(); ribbon.header.frame_id = "map"; ribbon.header.stamp = now
        ribbon.ns = "teacher_path_surface"; ribbon.id = 1; ribbon.type = Marker.TRIANGLE_LIST
        ribbon.action = Marker.ADD; ribbon.pose.orientation.w = 1.0
        ribbon.color = self._speed_color(speed); ribbon.lifetime = rospy.Duration(.35)
        half_width = 1.0
        stride = max(1, int(len(points) / 180))
        sampled = points[::stride]
        if not np.array_equal(sampled[-1], points[-1]): sampled = np.vstack((sampled, points[-1]))
        sides = []
        for i, p in enumerate(sampled):
            a = sampled[max(i-1, 0), :2]; b = sampled[min(i+1, len(sampled)-1), :2]
            tangent = b-a; norm = max(float(np.linalg.norm(tangent)), 1e-6)
            normal = np.asarray([-tangent[1], tangent[0]])/norm
            sides.append((p[:2]+half_width*normal, p[:2]-half_width*normal, float(p[2])+.08))
        for i in range(len(sides)-1):
            l0,r0,z0=sides[i]; l1,r1,z1=sides[i+1]
            for xy,z in ((l0,z0),(r0,z0),(l1,z1),(l1,z1),(r0,z0),(r1,z1)):
                ribbon.points.append(Point(float(xy[0]),float(xy[1]),z))
        array.markers.append(ribbon)

        ego = self.odom.pose.pose.position
        signal = self.signal_diagnostic
        ped = self.pedestrian_diagnostic
        reasons = []
        if data.get("walker_hazard", False): reasons.append("PEDESTRIAN COLLISION")
        if data.get("vehicle_hazard", False): reasons.append("NPC COLLISION")
        if data.get("changed_route", False): reasons.append("STATIC OBSTACLE AVOID")
        if data.get("light_hazard", False): reasons.append("TRAFFIC SIGNAL")
        if data.get("goal_stop", False): reasons.append("ROUTE GOAL")
        elif data.get("goal_slowdown", False):
            reasons.append("GOAL APPROACH %.1fm" % data.get("goal_distance", 0.0))
        if ped and not data.get("walker_hazard", False): reasons.append("PEDESTRIAN CAUTION")
        if not reasons: reasons.append("CLEAR")
        hud = Marker(); hud.header.frame_id="map"; hud.header.stamp=now; hud.ns="teacher_decision"; hud.id=2
        hud.type=Marker.TEXT_VIEW_FACING; hud.action=Marker.ADD; hud.pose.orientation.w=1.0
        hud.pose.position=Point(ego.x,ego.y,ego.z+4.5); hud.scale.z=1.0; hud.lifetime=rospy.Duration(.35)
        hud.color = ColorRGBA(1.0,.2,.15,1.0) if behavior == "STOP" else (ColorRGBA(1.0,.65,.05,1.0) if behavior == "AVOID" else ColorRGBA(.1,1.0,.35,1.0))
        hud.text="TEACHER: %s | %.1f km/h\n%s" % (behavior, speed*3.6, " + ".join(reasons))
        if signal:
            hud.text += "\nSIGNAL %s [%s] code=%d d=%.1fm%s" % (signal["light_id"], signal["movement"], signal["status"], signal["distance"], " (IGNORE RIGHT)" if signal["ignored"] else "")
            if not signal["ignored"]:
                hud.text += " | stop %.1fm before (%s)" % (signal.get("stop_buffer", self.traffic_stop_buffer), "CAMERA" if signal.get("camera_based", False) else "STOP_LINE")
        if ped:
            if ped.get("front_only", False):
                hud.text += "\nPED #%d %s d=%.1fm caution=%.1fkm/h" % (ped["actor_id"], "STATIONARY" if ped.get("stationary", False) else "AHEAD", ped["distance"],ped["cap"]*3.6)
            else:
                hud.text += "\nPED #%d CONFLICT d=%.1fm clearance=%.1fm cap=%.1fkm/h" % (ped["actor_id"],ped["distance"],ped["clearance"],ped["cap"]*3.6)
        array.markers.append(hud)

        if signal and not signal["ignored"]:
            idx=min(int(signal["route_index"]),len(self.agent._waypoint_planner.original_route_points)-1)
            target_idx=max(self.agent._waypoint_planner.route_index,
                           int(signal.get("stop_target_index", idx-int(self.traffic_stop_buffer*self.agent._waypoint_planner.points_per_meter))))
            target=self.agent._waypoint_planner.original_route_points[target_idx]
            mark=Marker(); mark.header.frame_id="map"; mark.header.stamp=now; mark.ns="signal_target"; mark.id=3
            mark.type=Marker.CYLINDER; mark.action=Marker.ADD; mark.pose.orientation.w=1.0
            mark.pose.position=Point(float(target[0]),float(target[1]),float(target[2])+.15)
            mark.scale.x=mark.scale.y=2.8; mark.scale.z=.25; mark.lifetime=rospy.Duration(.35)
            mark.color=ColorRGBA(.1,1,.1,.8) if signal["permitted"] else ColorRGBA(1,.05,.02,.9)
            array.markers.append(mark)
        if ped:
            forecast=Marker(); forecast.header.frame_id="map"; forecast.header.stamp=now; forecast.ns="pedestrian_prediction"; forecast.id=4
            forecast.type=Marker.LINE_STRIP; forecast.action=Marker.ADD; forecast.pose.orientation.w=1.0
            forecast.scale.x=.35; forecast.color=ColorRGBA(1,.45,.02,1); forecast.lifetime=rospy.Duration(.35)
            forecast.points=[Point(float(p[0]),float(p[1]),ego.z+.35) for p in ped["future"]]
            array.markers.append(forecast)
        return array

    def _publish(self, points, speed, behavior, hard_stop=False, data=None):
        now = rospy.Time.now(); msg = Path(); msg.header.stamp = now; msg.header.frame_id = "map"
        if len(points) > 1:
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1))]
            points = points[arc <= self.horizon]
        for i, p in enumerate(points):
            pose = PoseStamped(); pose.header = msg.header; pose.pose.position.x=float(p[0]); pose.pose.position.y=float(p[1]); pose.pose.position.z=float(p[2])
            j=min(i+1,len(points)-1); k=max(0,i-1); yaw=math.atan2(points[j,1]-points[k,1],points[j,0]-points[k,0])
            q=quaternion_from_euler(0,0,yaw); pose.pose.orientation.x=q[0]; pose.pose.orientation.y=q[1]; pose.pose.orientation.z=q[2]; pose.pose.orientation.w=q[3]
            msg.poses.append(pose)
        self.path_pub.publish(msg); self.speed_pub.publish(Float32(max(0., float(speed))))
        self.behavior_pub.publish(String(behavior))
        # The existing MPC contract is exactly [STOP, DRIVE]. Approach a
        # signal using the planned target speed, then assert STOP=1 once the
        # teacher actually decides to stop. The MPC maps this score to
        # accel=0/brake=1 and holds the vehicle at the virtual stop target.
        # That target deliberately remains before the mapped stop line so the
        # front camera retains a view of the traffic signal.
        full_stop = behavior == "STOP"
        scores = [1., 0.] if full_stop else [0., 1.]
        self.mode_pub.publish(Float32MultiArray(data=scores)); self.ready_pub.publish(Bool(True))
        self.marker_pub.publish(self._decision_markers(points, speed, behavior, hard_stop, data or {}))

    def _tick(self, _event):
        with self.lock:
            odom, ego, objects, path = self.odom, self.ego, self.objects, self.global_path
            if odom is None or path is None or len(path.poses)<2:
                self.ready_pub.publish(Bool(False)); return
            signature=(len(path.poses), round(path.poses[-1].pose.position.x,2), round(path.poses[-1].pose.position.y,2))
            try:
                # /global_path can be republished as a rolling/reordered path.
                # Reinitializing here resets upstream route_index to the route
                # beginning and makes MPC reject a far-away path. Initialize
                # once per process; an intentional route change restarts this
                # adapter through roslaunch.
                if self.agent is None:
                    if not self._initialize(path): return
                self.agent._waypoint_planner.speed_limits[:] = max(self.speed_limit, .1)
                self._update_world(odom, ego, objects); self._update_lights()
                imu=np.asarray([0.,0.,self._yaw(odom)])
                _, data=self.agent._get_control({"imu":(None,imu)}, plant=False)
                route=np.asarray(self.agent.remaining_route)
                planner = self.agent._waypoint_planner
                bootstrap_index_limit = max(
                    10, int(5.0 * planner.points_per_meter))
                # Empty remaining_route is recoverable only during initial
                # closed-loop bootstrap. At the real route end it is the goal
                # condition and must never refill the complete route.
                if (len(route) < 2 and not self.route_bootstrap_recovered and
                        int(planner.route_index) <= bootstrap_index_limit):
                    bootstrap_count = min(
                        len(planner.original_route_points),
                        max(2, int(100.0 * planner.points_per_meter)))
                    ego_xy = np.asarray([
                        odom.pose.pose.position.x,
                        odom.pose.pose.position.y])
                    recovered_index = int(np.argmin(np.linalg.norm(
                        planner.original_route_points[
                            :bootstrap_count, :2] - ego_xy, axis=1)))
                    planner.route_points = planner.original_route_points.copy()
                    planner.route_index = recovered_index
                    planner.last_route_index = recovered_index
                    self.route_bootstrap_recovered = True
                    rospy.logwarn(
                        "Recovered empty closed-loop PDM route at index %d",
                        recovered_index)
                    _, data = self.agent._get_control(
                        {"imu": (None, imu)}, plant=False)
                    route = np.asarray(self.agent.remaining_route)
                speed=float(data["target_speed"])
                route_index = min(int(planner.route_index),
                                  len(planner.original_route_points) - 1)
                remaining_points = planner.original_route_points[route_index:]
                if len(remaining_points) > 1:
                    goal_distance = float(np.linalg.norm(
                        np.diff(remaining_points[:, :2], axis=0), axis=1).sum())
                else:
                    goal_distance = 0.0
                at_goal = (route_index >= len(planner.original_route_points) -
                           1 - self.goal_index_margin or
                           goal_distance <= self.goal_stop_distance)
                if at_goal:
                    speed = 0.0
                    data["goal_stop"] = True
                elif goal_distance <= self.goal_slowdown_distance:
                    # Braking envelope v^2 = 2*a*d. This makes the teacher
                    # stop at the final waypoint instead of passing it.
                    stop_room = max(0.0, goal_distance - self.goal_stop_distance)
                    speed = min(speed, math.sqrt(2.0 * self.goal_decel * stop_room))
                    data["goal_slowdown"] = True
                    data["goal_distance"] = goal_distance
                speed=self._pedestrian_speed_cap(objects, route, speed)
                if self.pedestrian_hazard_full_stop and data.get("walker_hazard", False):
                    speed = 0.0
                elif data.get("changed_route", False):
                    # Keep enough speed to complete the maneuver, but do not
                    # carry the full MGeo cruise speed through an AVOID path.
                    speed = min(speed, self.static_avoid_speed)
                behavior="STOP" if speed<.05 else ("AVOID" if data.get("changed_route") else "DRIVE")
                hard_stop = bool(data.get("walker_hazard", False) or
                                 data.get("vehicle_hazard", False) or
                                 data.get("goal_stop", False))
                self._publish(route, speed, behavior, hard_stop, data)
            except Exception as exc:
                rospy.logerr_throttle(1.0, "Upstream PDM step failed: %s", exc)
                self.ready_pub.publish(Bool(False))


if __name__ == "__main__":
    MoraiUpstreamPDM(); rospy.spin()
