#!/usr/bin/env python3
"""CollisionData → morai_msgs/CollisionData  |  UDP 9092"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[2]))

import rospy
from std_msgs.msg import Header
from morai_msgs.msg import CollisionData, CollisionObject
from geometry_msgs.msg import Vector3
from lib.network.UDP import Receiver
from lib.define.CollisionData import CollisionData as CollisionDataDef

IP    = '127.0.0.1'
PORT  = 9092
TOPIC = '/morai/collision_data'


def build_collision_object(d):
    obj = CollisionObject()
    obj.objType = int(d.objType)
    obj.obj_id = int(d.obj_id)
    obj.pose = Vector3(x=float(d.pose_x), y=float(d.pose_y), z=float(d.pose_z))
    obj.globalOffset = Vector3(x=float(d.globalOffset_x), y=float(d.globalOffset_y), z=float(d.globalOffset_z))
    return obj


def build_collision_msg(raw, stamp):
    msg = CollisionData()
    msg.header = Header(stamp=stamp, frame_id='map')
    msg.collision_object = [build_collision_object(raw._data[i]) for i in range(5)]
    return msg


def main():
    rospy.init_node('morai_collision_data_publisher', anonymous=False)
    port  = rospy.get_param('~port', PORT)
    topic = rospy.get_param('~topic', TOPIC)
    pub   = rospy.Publisher(topic, CollisionData, queue_size=10)
    receiver = Receiver(IP, port, CollisionDataDef())
    rospy.loginfo(f"[CollisionData] UDP {IP}:{port} → {topic} (event-driven)")
    try:
        while not rospy.is_shutdown():
            try:
                raw = receiver._queue.get(timeout=0.5)
            except Exception:
                continue
            if raw.sec == 0:
                rospy.logwarn_throttle(5.0, "[CollisionData] 수신 대기 중…")
                continue
            pub.publish(build_collision_msg(raw, rospy.Time.now()))
    finally:
        receiver.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
