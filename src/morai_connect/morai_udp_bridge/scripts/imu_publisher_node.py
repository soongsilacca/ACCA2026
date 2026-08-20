#!/usr/bin/env python3
"""IMU → sensor_msgs/Imu  |  UDP 1112"""
import sys, math
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[2]))

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from lib.network.UDP import Receiver
from lib.define.IMU import IMU

IP       = '127.0.0.1'
PORT     = 1112
TOPIC    = '/imu'
FRAME_ID = 'imu'

_COV_ORI  = [0.01, 0, 0,  0, 0.01, 0,  0, 0, 0.01]  # orientation (roll, pitch, yaw)
_COV_GYRO = [0.01, 0, 0,  0, 0.01, 0,  0, 0, 0.01]  # angular velocity
_COV_ACCL = [0.1,  0, 0,  0, 0.1,  0,  0, 0, 0.1 ]  # linear acceleration


def _safe(val, default=0.0):
    return val if math.isfinite(val) else default


def build_imu_msg(imu, stamp):
    msg = Imu()
    msg.header = Header(stamp=stamp, frame_id=FRAME_ID)

    msg.orientation.w = imu.ori_w
    msg.orientation.x = imu.ori_x
    msg.orientation.y = imu.ori_y
    msg.orientation.z = imu.ori_z
    msg.orientation_covariance = list(_COV_ORI)

    msg.angular_velocity.x = _safe(imu.ang_vel_x)
    msg.angular_velocity.y = _safe(imu.ang_vel_y)
    msg.angular_velocity.z = _safe(imu.ang_vel_z)
    msg.angular_velocity_covariance = list(_COV_GYRO)

    msg.linear_acceleration.x = _safe(imu.lin_acc_x)
    msg.linear_acceleration.y = _safe(imu.lin_acc_y)
    msg.linear_acceleration.z = _safe(imu.lin_acc_z)
    msg.linear_acceleration_covariance = list(_COV_ACCL)
    return msg


def main():
    rospy.init_node('morai_imu_publisher', anonymous=False)
    port  = rospy.get_param('~port', PORT)
    topic = rospy.get_param('~topic', TOPIC)
    pub   = rospy.Publisher(topic, Imu, queue_size=10)
    sensor_imu = Receiver(IP, port, IMU())
    rospy.loginfo(f"[IMU] UDP {IP}:{port} → {topic} (event-driven)")
    try:
        while not rospy.is_shutdown():
            # 새 UDP 패킷이 도착할 때까지 블로킹 대기
            try:
                imu_data = sensor_imu._queue.get(timeout=0.5)
            except Exception:
                continue
            if (imu_data.ori_w == 0.0 and imu_data.ori_x == 0.0
                    and imu_data.ori_y == 0.0 and imu_data.ori_z == 0.0):
                rospy.logwarn_throttle(5.0, "[IMU] 수신 대기 중…")
                continue
            # rospy.Time.now() = 현재 /clock 시간 사용 → ego_vehicle_status와 동일한 시간 기준
            # UDP 패킷 내장 시간은 ego_status 패킷보다 ~30ms 늦게 도달 → TF 과거 조회 경고 방지
            pub.publish(build_imu_msg(imu_data, rospy.Time.now()))
    finally:
        sensor_imu.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
