"""MORAI-independent pieces ported from CARLA Garage PDM-Lite.

Upstream: autonomousvision/carla_garage, branch leaderboard_2,
commit 72f39a63423a5edef6904b1487e0360a64bcf445 (MIT).
The IDM integration follows team_code/autopilot.py::_compute_target_speed_idm.
CARLA World/Actor/Waypoint calls are intentionally handled by the ROS adapter.
"""
import math

import numpy as np
from scipy.integrate import RK45


def idm_target_speed(desired_speed, actor_length, ego_speed, actor_speed,
                     distance, minimum_distance=4.0, time_headway=0.25):
    desired_speed = max(float(desired_speed), 0.1)
    ego_speed = max(float(ego_speed), 0.0)
    actor_speed = max(float(actor_speed), 0.0)
    acceleration = 24.0
    braking = 3.72 if ego_speed > 6.02 else 8.7
    exponent = 4.0

    def equations(time, state):
        position, speed = state
        speed_difference = speed - actor_speed
        desired_gap = (minimum_distance + speed * time_headway +
                       speed * speed_difference /
                       (2.0 * math.sqrt(acceleration * braking)))
        gap = max(0.1, distance + time * actor_speed - position - actor_length)
        derivative = acceleration * (1.0 - (speed / desired_speed) ** exponent -
                                     (desired_gap / gap) ** 2)
        return [speed, derivative]

    integrator = RK45(equations, 0.0, [0.0, ego_speed], 0.05)
    while integrator.status == "running":
        integrator.step()
    return float(np.clip(integrator.y[1], 0.0, desired_speed))


def smooth_shift(reference, normal, offset, transition_length):
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(reference, axis=0), axis=1))]
    blend = np.clip(arc / max(float(transition_length), 0.1), 0.0, 1.0)
    blend = blend * blend * (3.0 - 2.0 * blend)
    return reference + normal * (float(offset) * blend)[:, None]


def oriented_box(center, yaw, length, width):
    local = np.asarray([[length, width], [length, -width],
                        [-length, -width], [-length, width]], dtype=float) * 0.5
    rotation = np.asarray([[math.cos(yaw), -math.sin(yaw)],
                           [math.sin(yaw), math.cos(yaw)]])
    return local.dot(rotation.T) + np.asarray(center, dtype=float)


def boxes_overlap(first, second):
    """Separating-axis test used for time-indexed occupancy collision."""
    for polygon in (first, second):
        for index in range(4):
            edge = polygon[(index + 1) % 4] - polygon[index]
            axis = np.asarray([-edge[1], edge[0]])
            norm = np.linalg.norm(axis)
            if norm < 1e-9:
                continue
            axis /= norm
            first_projection, second_projection = first.dot(axis), second.dot(axis)
            if first_projection.max() < second_projection.min() or second_projection.max() < first_projection.min():
                return False
    return True


def interpolate_path(path, stations):
    """Sample an xy path at longitudinal stations."""
    path = np.asarray(path, dtype=float)
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    return np.column_stack((np.interp(stations, arc, path[:, 0]),
                            np.interp(stations, arc, path[:, 1])))


def rollout_speed(initial_speed, target_speed, horizon, dt,
                  maximum_acceleration=2.5, comfortable_brake=4.0):
    """Longitudinal rollout used by every PDM proposal."""
    count = max(2, int(math.ceil(float(horizon) / float(dt))) + 1)
    times = np.arange(count, dtype=float) * float(dt)
    speeds = np.empty(count, dtype=float)
    stations = np.zeros(count, dtype=float)
    speeds[0] = max(float(initial_speed), 0.0)
    target_speed = max(float(target_speed), 0.0)
    for index in range(1, count):
        error = target_speed - speeds[index - 1]
        acceleration = np.clip(error / max(float(dt), 1e-3),
                               -comfortable_brake, maximum_acceleration)
        speeds[index] = max(0.0, speeds[index - 1] + acceleration * dt)
        stations[index] = stations[index - 1] + .5 * (
            speeds[index - 1] + speeds[index]) * dt
    return times, stations, speeds


def evaluate_proposal(path, initial_speed, target_speed, actors,
                      ego_length, ego_width, safety_margin,
                      horizon=4.0, dt=0.2):
    """Evaluate a MORAI trajectory with PDM-style time-indexed occupancy.

    ``actors`` is the ROS adapter representation. Moving NPCs and pedestrians
    are propagated with constant velocity; static obstacles remain fixed.
    """
    times, stations, speeds = rollout_speed(initial_speed, target_speed,
                                            horizon, dt)
    path_arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    stations = np.minimum(stations, path_arc[-1])
    positions = interpolate_path(path, stations)
    delta = np.gradient(positions, axis=0)
    yaws = np.arctan2(delta[:, 1], delta[:, 0])

    collision = False
    first_collision_time = float("inf")
    min_clearance = float("inf")
    closest_station = float("inf")
    closest_time = float("inf")
    closest = None
    collision_actor = None
    for actor in actors:
        actor_speed = float(np.linalg.norm(actor["v"]))
        # ObjectInfo heading is more stable at speed; velocity direction is
        # preferable when it disagrees while the actor is moving.
        actor_yaw = actor["yaw"]
        if actor_speed > .3:
            actor_yaw = math.atan2(actor["v"][1], actor["v"][0])
        initial_relative = actor["p"] - positions[0]
        initial_forward = np.asarray([math.cos(yaws[0]), math.sin(yaws[0])])
        initial_normal = np.asarray([-initial_forward[1], initial_forward[0]])
        initial_longitudinal = float(initial_relative.dot(initial_forward))
        initial_lateral = float(initial_relative.dot(initial_normal))
        # When stopped alongside a static object, every shifted proposal has
        # the same t=0 pose. Treating that existing side overlap as a future
        # collision deadlocks the vehicle forever. It is safe to score the
        # escape trajectory when the object is outside the ego center strip
        # and no farther forward than the front bumper.
        alongside_static = (actor["kind"] == "obstacle" and
                            abs(initial_longitudinal) <= .5 * ego_length and
                            abs(initial_lateral) > .5 * ego_width)
        initial_ego_box = oriented_box(positions[0], yaws[0],
                                       ego_length, ego_width)
        initial_actor_box = oriented_box(actor["p"], actor_yaw,
                                         actor["length"], actor["width"])
        existing_static_overlap = (actor["kind"] == "obstacle" and
                                   boxes_overlap(initial_ego_box,
                                                 initial_actor_box))
        for index, time_value in enumerate(times):
            actor_position = actor["p"] + actor["v"] * time_value
            center_distance = float(np.linalg.norm(positions[index] - actor_position))
            clearance = center_distance - .5 * ego_width - .5 * actor["width"] - safety_margin
            if clearance < min_clearance:
                min_clearance = clearance
                closest_station = float(stations[index])
                closest_time = float(time_value)
                closest = actor
            ego_box = oriented_box(positions[index], yaws[index],
                                   ego_length + 2.0 * safety_margin,
                                   ego_width + 2.0 * safety_margin)
            actor_box = oriented_box(actor_position, actor_yaw,
                                     actor["length"], actor["width"])
            if boxes_overlap(ego_box, actor_box):
                if alongside_static or existing_static_overlap:
                    continue
                collision = True
                if time_value < first_collision_time:
                    first_collision_time = float(time_value)
                    collision_actor = actor
                break

    relevant = collision_actor if collision_actor is not None else closest
    return {
        "collision": collision,
        "collision_time": first_collision_time,
        "clearance": min_clearance,
        "object_distance": closest_station,
        "closest_time": closest_time,
        "object_kind": "none" if relevant is None else relevant["kind"],
        "object_id": -1 if relevant is None else relevant["id"],
        "object_speed": 0.0 if relevant is None else float(np.linalg.norm(relevant["v"])),
        "object_length": 0.0 if relevant is None else relevant["length"],
        "progress": float(stations[-1]),
        "terminal_speed": float(speeds[-1]),
    }


def proposal_cost(meta, lateral_offset, target_speed, desired_speed, weights):
    """Lower-is-better PDM proposal score."""
    clearance_cost = (0.0 if not np.isfinite(meta["clearance"])
                      else 1.0 / max(meta["clearance"] + .2, .05))
    speed_error = abs(float(desired_speed) - float(target_speed))
    return (float(weights.get("collision", 1e6)) * float(meta["collision"])
            + float(weights.get("clearance", 80.0)) * clearance_cost
            + float(weights.get("lateral", 3.0)) * abs(float(lateral_offset))
            + float(weights.get("comfort", 4.0)) * float(lateral_offset) ** 2
            + float(weights.get("speed", 1.0)) * speed_error
            - float(weights.get("progress", 2.0)) * meta["progress"])
