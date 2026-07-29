#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF Local Initializer (ROS1)

Subscribes to /imu to get initial robot orientation.
Sets initial pose of EKF Local via /set_pose_local.

Race condition fix:
  - ekf_se_local 노드가 아직 subscribe 준비 안 된 상태에서 단 한 번만
    publish하면 메시지가 유실된다.
  - 대신, IMU 첫 메시지에서 즉시 pose를 계산한 뒤
    타이머로 ekf_se_local 구독자가 실제로 연결될 때까지 재전송한다.
  - 구독자가 연결되면 한 번 더 확실히 보내고 종료한다.
"""

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


class EKFLocalInitializer:
    # 재전송 주기 (초) — ekf 노드 startup 전 유실 방지
    RETRY_HZ = 5.0
    # 구독자 연결 확인 후 몇 번 더 전송할지
    CONFIRM_SENDS = 3

    def __init__(self):
        rospy.init_node('ekf_local_initializer', anonymous=False)

        self.pose_msg = None          # 초기화할 pose (계산 완료 후 고정)
        self.confirm_count = 0        # 구독자 연결 후 남은 전송 횟수

        # Latched publisher → ekf 노드가 나중에 연결되어도 수신 가능
        self.pub_set_pose = rospy.Publisher(
            '/set_pose_local', PoseWithCovarianceStamped, queue_size=1, latch=True
        )

        # IMU 구독 — 첫 유효 메시지에서 pose 계산
        self.sub_imu = rospy.Subscriber('/imu', Imu, self._imu_callback, queue_size=1)

        # 재전송 타이머 — pose 계산 전에는 아무것도 하지 않음
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.RETRY_HZ), self._retry_callback
        )

        rospy.loginfo("[EKF Local Init] Started. Waiting for /imu...")

    # ------------------------------------------------------------------
    def _imu_callback(self, msg: Imu):
        if self.pose_msg is not None:
            return  # 이미 pose 계산 완료

        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'odom'
        # position: odom 원점에서 시작
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.x = q.x
        pose_msg.pose.pose.orientation.y = q.y
        pose_msg.pose.pose.orientation.z = q.z
        pose_msg.pose.pose.orientation.w = q.w

        cov = [0.0] * 36
        cov[0]  = 0.01   # X (odom 원점, 확실)
        cov[7]  = 0.01   # Y
        cov[14] = 999.0  # Z (무시)
        cov[21] = 999.0  # Roll (무시)
        cov[28] = 999.0  # Pitch (무시)
        cov[35] = 0.05   # Yaw (IMU 신뢰)
        pose_msg.pose.covariance = cov

        self.pose_msg = pose_msg
        rospy.loginfo(
            f"[EKF Local Init] IMU received. Yaw={math.degrees(yaw):.1f}° "
            f"→ starting retry loop until ekf_se_local connects."
        )
        # IMU는 더 이상 필요 없음
        self.sub_imu.unregister()

    # ------------------------------------------------------------------
    def _retry_callback(self, event):
        if self.pose_msg is None:
            return  # 아직 IMU 수신 전

        n_subs = self.pub_set_pose.get_num_connections()

        # pose stamp는 항상 현재 시각으로 갱신 (EKF 타임스탬프 검증 통과)
        self.pose_msg.header.stamp = rospy.Time.now()
        self.pub_set_pose.publish(self.pose_msg)

        if n_subs > 0:
            self.confirm_count += 1
            rospy.loginfo(
                f"[EKF Local Init] Subscriber connected ({n_subs}). "
                f"Sent confirmation {self.confirm_count}/{self.CONFIRM_SENDS}."
            )
            if self.confirm_count >= self.CONFIRM_SENDS:
                rospy.loginfo("[EKF Local Init] Done. Node shutting down.")
                self.timer.shutdown()
                rospy.signal_shutdown("EKF Local Initialized successfully.")
        else:
            rospy.logwarn_throttle(
                2.0,
                "[EKF Local Init] Waiting for ekf_se_local to subscribe to /set_pose_local..."
            )


if __name__ == '__main__':
    try:
        EKFLocalInitializer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
