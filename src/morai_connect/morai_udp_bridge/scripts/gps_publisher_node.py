#!/usr/bin/env python3
"""GPS → sensor_msgs/NavSatFix  |  UDP 9090"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[2]))

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
from lib.network.UDP import Receiver
from lib.define.GPS import GPS

IP       = '127.0.0.1'
PORT     = 9090
TOPIC    = '/gps'
FRAME_ID = 'gps'
UERE_SIGMA = 10.0


def quality_to_nav_status(quality):
    try:
        q = int(quality)
    except (TypeError, ValueError):
        return NavSatStatus.STATUS_NO_FIX
    return {0: NavSatStatus.STATUS_NO_FIX,
            1: NavSatStatus.STATUS_FIX,
            2: NavSatStatus.STATUS_SBAS_FIX,
            4: NavSatStatus.STATUS_GBAS_FIX,
            5: NavSatStatus.STATUS_GBAS_FIX}.get(q, NavSatStatus.STATUS_FIX)


def build_navsatfix(gpgga):
    msg = NavSatFix()
    msg.header = Header(stamp=rospy.Time.now(), frame_id=FRAME_ID)
    msg.status.status  = quality_to_nav_status(gpgga.quality)
    msg.status.service = NavSatStatus.SERVICE_GPS

    msg.latitude  = gpgga.lat  if gpgga.lat_dir  in ('N', None) else -gpgga.lat
    msg.longitude = gpgga.lon  if gpgga.lon_dir  in ('E', None) else -gpgga.lon
    msg.altitude  = gpgga.alt

    try:
        hdop = float(gpgga.hdop)
    except (TypeError, ValueError):
        hdop = 1.0
    cov   = (hdop * UERE_SIGMA) ** 2
    cov_v = (hdop * UERE_SIGMA * 1.5) ** 2
    msg.position_covariance = [cov, 0, 0, 0, cov, 0, 0, 0, cov_v]
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return msg


def main():
    rospy.init_node('morai_gps_publisher', anonymous=False)
    port = rospy.get_param('~port', PORT)
    topic = rospy.get_param('~topic', TOPIC)
    pub = rospy.Publisher(topic, NavSatFix, queue_size=10)
    sensor_gps = Receiver(IP, port, GPS())
    rospy.loginfo(f"[GPS] UDP {IP}:{port} → {topic} (event-driven)")

    # UDP 패킷이 수신될 때마다 즉시 publish (ROS timer 없이 큐 블로킹 방식)
    last_seq = [None]

    try:
        while not rospy.is_shutdown():
            # _queue에서 새 패킷이 올 때까지 최대 0.5초 블로킹 대기
            try:
                raw = sensor_gps._queue.get(timeout=0.5)
            except Exception:
                continue  # timeout → 다시 대기

            # parsing 후 GPGGA 유효성 확인
            raw.parsing()
            gpgga = raw.gpgga
            if gpgga.lat == 0.0 and gpgga.lon == 0.0:
                rospy.logwarn_throttle(5.0, "[GPS] GPGGA 수신 대기 중…")
                continue

            pub.publish(build_navsatfix(gpgga))
    finally:
        sensor_gps.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
