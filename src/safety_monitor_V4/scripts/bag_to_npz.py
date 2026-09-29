#!/usr/bin/env python3
"""bag에서 충돌 판정 입력만 뽑아 .npz로 저장한다 (포인트클라우드는 뽑지 않음).

예)
  rosrun safety_monitor bag_to_npz.py lane_change.bag -o lane_change.npz
  rosrun safety_monitor bag_to_npz.py a.bag --ego-topic /rear_lidar/ego_odometry --ego-type odometry
"""
import argparse
import math

import numpy as np
import rosbag


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("-o", "--output")
    ap.add_argument("--tracks-topic", default="/rear_lidar/tracked_objects")
    ap.add_argument("--ego-topic", default="/morai/ego_vehicle_status")
    ap.add_argument("--ego-type", choices=["ego_status", "odometry"], default="ego_status")
    ap.add_argument("--ego-velocity-scale", type=float, default=1.0 / 3.6,
                    help="EgoVehicleStatus velocity.x 단위 변환 (km/h -> m/s)")
    ap.add_argument("--route-topic", default="/local_route")
    args = ap.parse_args()
    out = args.output or args.bag.rsplit(".", 1)[0] + ".npz"

    frame_t, rows, ego = [], [], []
    route_t, route_offsets, route_pts = [], [0], []
    route_frame = "base_link"
    topics = [args.tracks_topic, args.ego_topic, args.route_topic]
    with rosbag.Bag(args.bag) as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            t = t.to_sec()
            if topic == args.tracks_topic:
                fi = len(frame_t)
                frame_t.append(t)
                for o in msg.objects:
                    rows.append([fi, o.track_id, o.pose.position.x, o.pose.position.y,
                                 o.dimensions.x, o.dimensions.y, o.dimensions.z,
                                 o.velocity.x, o.velocity.y, o.is_dynamic,
                                 o.odometry_compensated, o.is_coasting, o.time_since_update])
            elif topic == args.ego_topic:
                if args.ego_type == "ego_status":
                    ego.append([t, msg.position.x, msg.position.y, math.radians(msg.yaw),
                                msg.velocity.x * args.ego_velocity_scale])
                else:
                    v = msg.twist.twist.linear
                    ego.append([t, msg.pose.pose.position.x, msg.pose.pose.position.y,
                                yaw_from_quat(msg.pose.pose.orientation), math.hypot(v.x, v.y)])
            elif topic == args.route_topic and msg.poses:
                route_frame = msg.header.frame_id or route_frame
                route_t.append(t)
                route_pts.extend([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
                route_offsets.append(len(route_pts))

    np.savez_compressed(
        out,
        trk_frame_t=np.asarray(frame_t, float),
        trk_rows=np.asarray(rows, float).reshape(-1, 13),
        ego=np.asarray(ego, float).reshape(-1, 5),
        route_t=np.asarray(route_t, float),
        route_offsets=np.asarray(route_offsets, int),
        route_pts=np.asarray(route_pts, float).reshape(-1, 2),
        route_frame=np.asarray(route_frame),
    )
    print("saved %s: frames=%d objects=%d ego=%d routes=%d (route_frame=%s)" % (
        out, len(frame_t), len(rows), len(ego), len(route_t), route_frame))


if __name__ == "__main__":
    main()
