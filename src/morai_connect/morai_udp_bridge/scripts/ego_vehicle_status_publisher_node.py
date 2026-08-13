#!/usr/bin/env python3
"""EgoVehicleStatus → morai_msgs/EgoVehicleStatus  |  UDP 9009"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[2]))

import rospy
import time
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from morai_msgs.msg import EgoVehicleStatus as EgoVehicleStatusMsg
from lib.network.UDP import Receiver
from lib.define.EgoVehicleStatus import EgoVehicleStatus as EgoVehicleStatusDef

IP       = '127.0.0.1'
PORT     = 9009
TOPIC    = '/morai/ego_vehicle_status'
FRAME_ID = 'base_link'


def build_ego_msg(raw, stamp):
    msg = EgoVehicleStatusMsg()
    msg.header    = Header(stamp=stamp, frame_id=FRAME_ID)
    msg.ctrl_mode = int(raw.ctrl_mode)
    msg.gear = int(raw.gear)
    msg.signed_vel = float(raw.signed_vel)
    msg.map_data_id = int(raw.map_data_id)
    msg.accel = float(raw.accel)
    msg.brake = float(raw.brake)
    msg.size = Vector3(x=float(raw.size_x), y=float(raw.size_y), z=float(raw.size_z))
    msg.overhang = float(raw.overhang)
    msg.wheelbase = float(raw.wheelbase)
    msg.rear_overhang = float(raw.rear_overhang)
    msg.position = Vector3(x=float(raw.pos_x), y=float(raw.pos_y), z=float(raw.pos_z))
    msg.roll = float(raw.roll)
    msg.pitch = float(raw.pitch)
    msg.yaw = float(raw.yaw)
    msg.velocity = Vector3(x=float(raw.vel_x), y=float(raw.vel_y), z=float(raw.vel_z))
    msg.angular_velocity = Vector3(x=float(raw.ang_vel_x), y=float(raw.ang_vel_y), z=float(raw.ang_vel_z))
    msg.acceleration = Vector3(x=float(raw.accel_x), y=float(raw.accel_y), z=float(raw.accel_z))
    msg.steer = float(raw.steer)
    msg.link_id = raw.link_id.decode('utf-8') if isinstance(raw.link_id, bytes) else str(raw.link_id)

    msg.tire_lateral_force_fl = float(raw.tire_lateral_force_fl)
    msg.tire_lateral_force_fr = float(raw.tire_lateral_force_fr)
    msg.tire_lateral_force_rl = float(raw.tire_lateral_force_rl)
    msg.tire_lateral_force_rr = float(raw.tire_lateral_force_rr)

    msg.side_slip_angle_fl = float(raw.side_slip_angle_fl)
    msg.side_slip_angle_fr = float(raw.side_slip_angle_fr)
    msg.side_slip_angle_rl = float(raw.side_slip_angle_rl)
    msg.side_slip_angle_rr = float(raw.side_slip_angle_rr)

    msg.tire_cornering_stiffness_fl = float(raw.tire_cornering_stiffness_fl)
    msg.tire_cornering_stiffness_fr = float(raw.tire_cornering_stiffness_fr)
    msg.tire_cornering_stiffness_rl = float(raw.tire_cornering_stiffness_rl)
    msg.tire_cornering_stiffness_rr = float(raw.tire_cornering_stiffness_rr)
    return msg


def main():
    rospy.init_node('morai_ego_vehicle_status_publisher', anonymous=False)
    port  = rospy.get_param('~port', PORT)
    topic = rospy.get_param('~topic', TOPIC)
    pub   = rospy.Publisher(topic, EgoVehicleStatusMsg, queue_size=10)

    use_sim_time = rospy.get_param('/use_sim_time', False)
    if use_sim_time:
        from rosgraph_msgs.msg import Clock
        clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)

    receiver = Receiver(IP, port, EgoVehicleStatusDef())
    rospy.loginfo(f"[EgoVehicleStatus] UDP {IP}:{port} → {topic} (event-driven, use_sim_time={use_sim_time})")
    try:
        while not rospy.is_shutdown():
            # 새 UDP 패킷이 도착할 때까지 블로킹 대기 (time.sleep 사용 → /clock 데드락 방지)
            try:
                raw = receiver._queue.get(timeout=0.5)
            except Exception:
                continue
            if raw.sec == 0:
                rospy.logwarn_throttle(5.0, "[EgoVehicleStatus] 수신 대기 중…")
                continue

            stamp = rospy.Time(raw.sec, raw.nsec) if use_sim_time else rospy.Time.now()

            if use_sim_time:
                clock_msg = Clock()
                clock_msg.clock = stamp
                clock_pub.publish(clock_msg)

            pub.publish(build_ego_msg(raw, stamp))
    finally:
        receiver.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
