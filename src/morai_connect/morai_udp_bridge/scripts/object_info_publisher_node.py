#!/usr/bin/env python3
"""MORAI ObjectInfo UDP -> morai_msgs/ObjectStatusList."""
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

import rospy
from geometry_msgs.msg import Vector3
from morai_msgs.msg import ObjectStatus, ObjectStatusList
from std_msgs.msg import Header

from lib.define.ObjectInfo import ObjectInfo
from lib.network.UDP import Receiver


def object_msg(raw, stamp):
    msg = ObjectStatus()
    msg.unique_id = int(raw.obj_id)
    msg.type = int(raw.objType)
    msg.name = "object_%d" % int(raw.obj_id)
    msg.heading = float(raw.heading)
    msg.position = Vector3(float(raw.pose_x), float(raw.pose_y), float(raw.pose_z))
    # MORAI ObjectInfo UDP velocity is km/h; internal planning uses m/s.
    msg.velocity = Vector3(float(raw.vel_x) / 3.6, float(raw.vel_y) / 3.6,
                           float(raw.vel_z) / 3.6)
    msg.acceleration = Vector3(float(raw.accel_x), float(raw.accel_y), float(raw.accel_z))
    msg.size = Vector3(float(raw.size_x), float(raw.size_y), float(raw.size_z))
    return msg


def main():
    rospy.init_node("morai_object_info")
    ip = rospy.get_param("~ip", "127.0.0.1")
    port = int(rospy.get_param("~port", 7505))
    topic = rospy.get_param("~topic", "/Object_topic")
    publisher = rospy.Publisher(topic, ObjectStatusList, queue_size=1)
    receiver = Receiver(ip, port, ObjectInfo())
    rospy.loginfo("[ObjectInfo] MORAI UDP 7605 -> %s:%d -> %s", ip, port, topic)
    try:
        while not rospy.is_shutdown():
            try:
                raw = receiver._queue.get(timeout=0.5)
            except Exception:
                rospy.logwarn_throttle(5.0, "[ObjectInfo] waiting for UDP packets on %s:%d", ip, port)
                continue
            stamp = rospy.Time.now(); result = ObjectStatusList()
            result.header = Header(stamp=stamp, frame_id="map")
            for item in raw.data:
                if int(item.obj_id) <= 0:
                    continue
                converted = object_msg(item, stamp)
                if int(item.objType) == 0:
                    result.pedestrian_list.append(converted)
                elif int(item.objType) == 1:
                    result.npc_list.append(converted)
                elif int(item.objType) == 2:
                    result.obstacle_list.append(converted)
            result.num_of_npcs = len(result.npc_list)
            result.num_of_pedestrian = len(result.pedestrian_list)
            result.num_of_obstacle = len(result.obstacle_list)
            publisher.publish(result)
    finally:
        receiver.stop()


if __name__ == "__main__":
    main()
