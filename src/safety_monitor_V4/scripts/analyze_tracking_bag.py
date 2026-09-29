#!/usr/bin/env python3
"""Summarize ego-motion consistency and false-dynamic rear LiDAR tracks."""

import argparse
import bisect
import math
import statistics

import rosbag
from tf.transformations import euler_from_quaternion


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))]


def median(values):
    return statistics.median(values) if values else float("nan")


def yaw_from_pose(pose):
    q = pose.orientation
    return euler_from_quaternion((q.x, q.y, q.z, q.w))[2]


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--turn-rate", type=float, default=0.03,
                        help="absolute yaw-rate threshold in rad/s")
    parser.add_argument("--details", action="store_true",
                        help="print a compact timeline for persistent tracks")
    parser.add_argument("--min-track-frames", type=int, default=8,
                        help="minimum frames shown with --details")
    parser.add_argument("--track-id", action="append", type=int, default=[],
                        help="print every published sample for this track ID")
    args = parser.parse_args()

    odom = []
    track_frames = []
    statuses = []
    with rosbag.Bag(args.bag) as bag:
        topics = ["/rear_lidar/ego_odometry",
                  "/rear_lidar/tracked_objects", "/rear_lidar/status"]
        for topic, msg, receipt in bag.read_messages(topics=topics):
            if topic == "/rear_lidar/ego_odometry":
                stamp = msg.header.stamp.to_sec() or receipt.to_sec()
                odom.append((stamp, msg.pose.pose.position.x,
                             msg.pose.pose.position.y, yaw_from_pose(msg.pose.pose),
                             msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                             msg.twist.twist.angular.z))
            elif topic == "/rear_lidar/tracked_objects":
                stamp = msg.header.stamp.to_sec() or receipt.to_sec()
                track_frames.append((stamp, msg.objects))
            elif topic == "/rear_lidar/status" and msg.data.startswith("OK "):
                statuses.append(msg.data)

    if not odom:
        raise SystemExit("no /rear_lidar/ego_odometry in bag")

    odom.sort()
    odom_times = [sample[0] for sample in odom]
    pose_speeds = []
    twist_speeds = []
    pose_yaw_rates = []
    reported_yaw_rates = []
    yaw_rate_errors = []
    for previous, current in zip(odom, odom[1:]):
        dt = current[0] - previous[0]
        if dt < 0.005 or dt > 0.2:
            continue
        pose_speed = math.hypot(current[1] - previous[1],
                                current[2] - previous[2]) / dt
        pose_yaw_rate = wrap_angle(current[3] - previous[3]) / dt
        reported_yaw_rate = current[6]
        if pose_speed < 80.0 and abs(pose_yaw_rate) < 3.0:
            pose_speeds.append(pose_speed)
            twist_speeds.append(math.hypot(current[4], current[5]))
            pose_yaw_rates.append(pose_yaw_rate)
            reported_yaw_rates.append(reported_yaw_rate)
            yaw_rate_errors.append(pose_yaw_rate - reported_yaw_rate)

    def nearest_odom(stamp):
        index = bisect.bisect_left(odom_times, stamp)
        choices = [i for i in (index - 1, index) if 0 <= i < len(odom)]
        return min((odom[i] for i in choices), key=lambda sample: abs(sample[0] - stamp))

    frame_stats = {"straight": [0, 0, 0, 0, 0], "turn": [0, 0, 0, 0, 0]}
    tracks = {}
    turn_gate_counts = {scale: 0 for scale in (0.0, 0.25, 0.5, 0.75, 1.0)}
    turn_gate_vehicle_counts = dict(turn_gate_counts)
    measured_objects = 0
    for stamp, objects in track_frames:
        ego = nearest_odom(stamp)
        phase = "turn" if abs(ego[6]) >= args.turn_rate else "straight"
        frame_stats[phase][0] += 1
        frame_stats[phase][1] += len(objects)
        frame_stats[phase][2] += sum(bool(obj.is_dynamic) for obj in objects)
        frame_stats[phase][3] += sum(bool(obj.is_dynamic) and
                                     not getattr(obj, "is_coasting", False)
                                     for obj in objects)
        frame_stats[phase][4] += sum(bool(obj.is_dynamic) and
                                     getattr(obj, "is_coasting", False)
                                     for obj in objects)
        for obj in objects:
            speed = math.hypot(obj.velocity.x, obj.velocity.y)
            object_range = math.hypot(obj.pose.position.x, obj.pose.position.y)
            object_length = max(obj.dimensions.x, obj.dimensions.y)
            if not getattr(obj, "is_coasting", False):
                measured_objects += 1
                for scale in turn_gate_counts:
                    threshold = 0.8 + scale * abs(ego[6]) * object_range
                    if speed >= threshold:
                        turn_gate_counts[scale] += 1
                        if 0.8 <= object_length <= 6.0:
                            turn_gate_vehicle_counts[scale] += 1
            item = tracks.setdefault(obj.track_id, {
                "frames": 0, "dynamic": 0, "speeds": [], "turn_rates": [],
                "dynamic_turn_rates": [],
                "lengths": [], "widths": [], "heights": [], "positions": [],
                "xs": [], "ys": [], "relative_speeds": [], "ego_speeds": [],
                "dynamic_stamps": [], "coasting": 0,
                "first": stamp, "last": stamp,
            })
            item["frames"] += 1
            item["dynamic"] += int(obj.is_dynamic)
            item["speeds"].append(speed)
            item["turn_rates"].append(abs(ego[6]))
            if obj.is_dynamic:
                item["dynamic_turn_rates"].append(abs(ego[6]))
                item["dynamic_stamps"].append(stamp)
            item["lengths"].append(max(obj.dimensions.x, obj.dimensions.y))
            item["widths"].append(min(obj.dimensions.x, obj.dimensions.y))
            item["heights"].append(obj.dimensions.z)
            item["positions"].append(math.hypot(obj.pose.position.x,
                                                 obj.pose.position.y))
            item["xs"].append(obj.pose.position.x)
            item["ys"].append(obj.pose.position.y)
            item["relative_speeds"].append(math.hypot(obj.relative_velocity.x,
                                                       obj.relative_velocity.y))
            item["ego_speeds"].append(math.hypot(ego[4], ego[5]))
            item["coasting"] += int(getattr(obj, "is_coasting", False))
            item["last"] = stamp

    dynamic_tracks = [(track_id, item) for track_id, item in tracks.items()
                      if item["dynamic"]]
    dynamic_tracks.sort(key=lambda pair: (pair[1]["dynamic"],
                                          max(pair[1]["speeds"])), reverse=True)

    print("EGO SAMPLES", len(odom))
    print("pose speed median/p90: %.3f / %.3f m/s" %
          (median(pose_speeds), percentile(pose_speeds, 0.90)))
    print("twist speed median/p90: %.3f / %.3f m/s" %
          (median(twist_speeds), percentile(twist_speeds, 0.90)))
    moving_pairs = [(p, r) for p, r in zip(pose_yaw_rates, reported_yaw_rates)
                    if abs(p) >= args.turn_rate or abs(r) >= args.turn_rate]
    print("turn samples", len(moving_pairs))
    if moving_pairs:
        ratios = [r / p for p, r in moving_pairs if abs(p) >= args.turn_rate]
        sign_matches = sum(p * r > 0 for p, r in moving_pairs)
        print("pose yaw-rate median/p90 abs: %.4f / %.4f rad/s" %
              (median([abs(p) for p, _ in moving_pairs]),
               percentile([abs(p) for p, _ in moving_pairs], 0.90)))
        print("reported yaw-rate median/p90 abs: %.4f / %.4f rad/s" %
              (median([abs(r) for _, r in moving_pairs]),
               percentile([abs(r) for _, r in moving_pairs], 0.90)))
        print("reported/pose yaw-rate ratio median: %.4f, sign match: %.1f%%" %
              (median(ratios), 100.0 * sign_matches / len(moving_pairs)))
        print("yaw-rate error median/p90 abs: %.4f / %.4f rad/s" %
              (median([abs(v) for v in yaw_rate_errors]),
               percentile([abs(v) for v in yaw_rate_errors], 0.90)))

    # Pose fields can be quantized more coarsely than the UDP publication rate.
    # A centered window gives a less noisy consistency check than one-sample
    # differentiation.
    window_rates = []
    half_window = 5
    for index in range(half_window, len(odom) - half_window):
        first = odom[index - half_window]
        last = odom[index + half_window]
        dt = last[0] - first[0]
        if dt <= 0.05 or dt > 0.5:
            continue
        pose_rate = wrap_angle(last[3] - first[3]) / dt
        reported = median([odom[i][6] for i in
                           range(index - half_window, index + half_window + 1)])
        if abs(pose_rate) >= args.turn_rate or abs(reported) >= args.turn_rate:
            window_rates.append((pose_rate, reported))
    if window_rates:
        ratios = [reported / pose for pose, reported in window_rates
                  if abs(pose) >= args.turn_rate]
        sign_matches = sum(pose * reported > 0
                           for pose, reported in window_rates)
        errors = [abs(pose - reported) for pose, reported in window_rates]
        print("windowed reported/pose ratio median: %.4f, sign match: %.1f%%, "
              "error median/p90: %.4f/%.4f rad/s" %
              (median(ratios), 100.0 * sign_matches / len(window_rates),
               median(errors), percentile(errors, 0.90)))

    print("\nTRACK FRAMES")
    for phase, values in frame_stats.items():
        frames, objects, dynamic, measured_dynamic, coast_dynamic = values
        print("%s: frames=%d objects=%d dynamic=%d measured_dynamic=%d "
              "coast_dynamic=%d dynamic/object=%.1f%%" %
              (phase, frames, objects, dynamic, measured_dynamic, coast_dynamic,
               100.0 * dynamic / max(objects, 1)))
    print("total tracks=%d tracks marked dynamic=%d" %
          (len(tracks), len(dynamic_tracks)))
    print("measured speed-gate candidates (all / 0.8m<=length<=6m) out of %d:" %
          measured_objects)
    for scale in turn_gate_counts:
        print("  turn_scale=%.2f: %d / %d" %
              (scale, turn_gate_counts[scale], turn_gate_vehicle_counts[scale]))
    print("\nTOP TRACKS MARKED DYNAMIC")
    for track_id, item in dynamic_tracks[:30]:
        print("id=%d frames=%d dyn=%d duration=%.2fs speed_med/max=%.2f/%.2f "
              "range_med=%.1f size_med=%.1fx%.1f yaw_rate_dyn_min/med/max="
              "%.3f/%.3f/%.3f coast=%d" %
              (track_id, item["frames"], item["dynamic"],
               item["last"] - item["first"], median(item["speeds"]),
               max(item["speeds"]), median(item["positions"]),
               median(item["lengths"]), median(item["widths"]),
               min(item["dynamic_turn_rates"]),
               median(item["dynamic_turn_rates"]),
               max(item["dynamic_turn_rates"]), item["coasting"]))

    if args.details:
        bag_start = min(stamp for stamp, _ in track_frames)
        persistent = [(track_id, item) for track_id, item in tracks.items()
                      if item["frames"] >= args.min_track_frames]
        persistent.sort(key=lambda pair: pair[1]["first"])
        print("\nPERSISTENT TRACK TIMELINE")
        for track_id, item in persistent:
            dynamic_first = (item["dynamic_stamps"][0] - bag_start
                             if item["dynamic_stamps"] else float("nan"))
            print("id=%d t=%.1f..%.1f frames=%d measured=%d dyn=%d dyn_first=%.1f "
                  "xy_med=(%.1f,%.1f) range=%.1f size=%.1fx%.1fx%.1f "
                  "ground/relative/ego_speed=%.1f/%.1f/%.1f yaw_rate=%.3f" %
                  (track_id, item["first"] - bag_start,
                   item["last"] - bag_start, item["frames"],
                   item["frames"] - item["coasting"], item["dynamic"],
                   dynamic_first, median(item["xs"]), median(item["ys"]),
                   median(item["positions"]), median(item["lengths"]),
                   median(item["widths"]), median(item["heights"]),
                   median(item["speeds"]), median(item["relative_speeds"]),
                   median(item["ego_speeds"]), median(item["turn_rates"])))

    sources = {}
    for status in statuses:
        marker = "motion_source="
        source = status.split(marker, 1)[1].split()[0] if marker in status else "unknown"
        sources[source] = sources.get(source, 0) + 1
    print("\nSTATUS motion sources", sources)

    if args.track_id:
        selected = set(args.track_id)
        bag_start = min(stamp for stamp, _ in track_frames)
        print("\nSELECTED TRACK SAMPLES")
        for stamp, objects in track_frames:
            ego = nearest_odom(stamp)
            for obj in objects:
                if obj.track_id not in selected:
                    continue
                speed = math.hypot(obj.velocity.x, obj.velocity.y)
                relative_speed = math.hypot(obj.relative_velocity.x,
                                             obj.relative_velocity.y)
                object_range = math.hypot(obj.pose.position.x,
                                          obj.pose.position.y)
                length = max(obj.dimensions.x, obj.dimensions.y)
                threshold = 0.8 + 0.5 * abs(ego[6]) * object_range
                print("id=%d t=%.2f xy=(%.2f,%.2f) size=%.2fx%.2fx%.2f "
                      "v=(%.2f,%.2f) speed=%.2f rv=(%.2f,%.2f) rel=%.2f "
                      "ego=%.2f yaw=%.3f gate=%.2f "
                      "dynamic=%d coast=%d age=%.2f since=%.2f" %
                      (obj.track_id, stamp - bag_start, obj.pose.position.x,
                       obj.pose.position.y, length,
                       min(obj.dimensions.x, obj.dimensions.y), obj.dimensions.z,
                       obj.velocity.x, obj.velocity.y, speed,
                       obj.relative_velocity.x, obj.relative_velocity.y,
                       relative_speed, math.hypot(ego[4], ego[5]),
                       abs(ego[6]), threshold, obj.is_dynamic,
                       getattr(obj, "is_coasting", False), obj.age,
                       obj.time_since_update))


if __name__ == "__main__":
    main()
