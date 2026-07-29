#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF Global Initializer & Reset Monitor (ROS1)

This node:
1. Subscribes to /gps_pose and /imu to perform the initial alignment of the
   global EKF (via /set_pose_global).
2. Runs continuously to monitor GPS blackout (timeout) and recovery, and
   sudden jumps (spawn/teleport).
3. Re-initializes (resets) the global EKF and NDT pose when recovery with
   large drift or a spawn event is detected.

Race condition fix:
  - /gps_pose (GPS node) 와 /imu 중 어느 쪽이 먼저 오더라도 init 가능.
  - 두 센서 중 하나가 수신되면 즉시 available로 표시하고, 나머지를
    최대 SENSOR_WAIT_TIMEOUT 초 기다린 뒤 available한 것으로만 init.
  - EKF 구독자가 연결될 때까지 재전송 타이머 사용 (blocking sleep 없음).
"""

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class EKFGlobalInitializer:
    # 두 번째 센서를 기다리는 최대 시간 (초) — 이후 수신된 것만으로 init
    SENSOR_WAIT_TIMEOUT = 3.0
    # init 완료 후 EKF 구독자 연결까지 재전송 주기 (Hz)
    RETRY_HZ = 5.0
    # 구독자 연결 확인 후 몇 번 더 전송할지
    CONFIRM_SENDS = 3

    def __init__(self):
        rospy.init_node('ekf_global_initializer', anonymous=False)

        # ── State ─────────────────────────────────────────────────────
        self.initialized     = False
        self.gps_pose_received = False
        self.imu_received    = False
        self._first_sensor_time = None   # 첫 번째 센서 수신 시각

        self.initial_x = 0.0
        self.initial_y = 0.0

        # Continuous monitoring
        self.latest_q   = None
        self.latest_yaw = 0.0
        self.latest_ekf_pose = None

        self.last_gps_x    = 0.0
        self.last_gps_y    = 0.0
        self.last_gps_time = rospy.Time(0)
        self.is_gps_active = False

        self.last_reset_time = rospy.Time(0)
        self.reset_cooldown  = 3.0  # seconds

        # init 완료 후 재전송 카운터
        self._confirm_count = 0
        self._init_pose_msg = None  # 재전송용 pose 캐시

        # ── Parameters ────────────────────────────────────────────────
        self.gps_timeout               = rospy.get_param('~gps_timeout',               3.0)
        self.jump_threshold            = rospy.get_param('~jump_threshold',             5.0)
        self.recovery_drift_threshold  = rospy.get_param('~recovery_drift_threshold',   5.0)

        # ── Publishers ────────────────────────────────────────────────
        self.pub_set_pose = rospy.Publisher(
            '/set_pose_global', PoseWithCovarianceStamped, queue_size=1, latch=True
        )

        # ── Subscribers ───────────────────────────────────────────────
        self.sub_gps_pose = rospy.Subscriber(
            '/gps_pose', PoseWithCovarianceStamped, self.gps_pose_callback, queue_size=1
        )
        self.sub_imu = rospy.Subscriber(
            '/imu', Imu, self.imu_callback, queue_size=1
        )
        self.sub_ekf = rospy.Subscriber(
            '/localization/kinematic_state', Odometry, self.ekf_callback, queue_size=1
        )

        # ── Timers ────────────────────────────────────────────────────
        # GPS 타임아웃 감시
        self.monitor_timer = rospy.Timer(rospy.Duration(1.0), self.check_gps_timeout)
        # init 재전송 (init 전에는 no-op)
        self.retry_timer = rospy.Timer(
            rospy.Duration(1.0 / self.RETRY_HZ), self._retry_callback
        )

        rospy.loginfo(
            f"[EKF Global Init] Started.\n"
            f"  Sensor wait timeout : {self.SENSOR_WAIT_TIMEOUT}s\n"
            f"  GPS Timeout         : {self.gps_timeout}s\n"
            f"  Jump Threshold      : {self.jump_threshold}m\n"
            f"  Recovery Drift Thr  : {self.recovery_drift_threshold}m"
        )

    # ── IMU callback ─────────────────────────────────────────────────
    def imu_callback(self, msg: Imu):
        self.latest_q = msg.orientation
        _, _, yaw = euler_from_quaternion(
            [self.latest_q.x, self.latest_q.y, self.latest_q.z, self.latest_q.w]
        )
        self.latest_yaw = yaw

        if not self.imu_received:
            self.imu_received = True
            if self._first_sensor_time is None:
                # use_sim_time=true 환경에서 rospy.Time.now()가 0을 반환할 수 있음.
                # msg.header.stamp을 우선 사용하고, 그것도 0이면 유예.
                t = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
                if t.to_sec() > 0:
                    self._first_sensor_time = t
            rospy.loginfo(
                f"[EKF Global Init] /imu received. Yaw={math.degrees(yaw):.1f}°"
            )
        self._try_initialize()

    # ── EKF output callback ──────────────────────────────────────────
    def ekf_callback(self, msg: Odometry):
        self.latest_ekf_pose = msg.pose.pose

    # ── GPS pose callback ────────────────────────────────────────────
    def gps_pose_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()

        # 시간 역행 감지 (bag 루프 등)
        if now.to_sec() < self.last_gps_time.to_sec():
            rospy.logwarn("[EKF Global Init] Time went backwards! Resetting state.")
            self.initialized        = False
            self.gps_pose_received  = False
            self.is_gps_active      = False
            self.latest_ekf_pose    = None
            self.last_reset_time    = rospy.Time(0)
            self._confirm_count     = 0
            self._init_pose_msg     = None

        if not self.initialized:
            if not self.gps_pose_received:
                self.gps_pose_received = True
                self.initial_x = x
                self.initial_y = y
                if self._first_sensor_time is None:
                    # use_sim_time=true 환경에서 rospy.Time.now()가 0을 반환할 수 있음.
                    # msg.header.stamp을 우선 사용하고, 그것도 0이면 유예.
                    t = now if now.to_sec() > 0 else rospy.Time.now()
                    if t.to_sec() > 0:
                        self._first_sensor_time = t
                rospy.loginfo(
                    f"[EKF Global Init] /gps_pose received. "
                    f"pos=({x:.2f}, {y:.2f})"
                )
            self._try_initialize()
            if self.initialized:
                self.last_gps_x    = x
                self.last_gps_y    = y
                self.last_gps_time = now
                self.is_gps_active = True
            return

        # ── 초기화 완료 후: 연속 모니터링 ────────────────────────────

        # 1. GPS 블랙아웃 복구
        if not self.is_gps_active:
            drift = 0.0
            if self.latest_ekf_pose is not None:
                ekf_x  = self.latest_ekf_pose.position.x
                ekf_y  = self.latest_ekf_pose.position.y
                drift  = math.sqrt((x - ekf_x)**2 + (y - ekf_y)**2)
            rospy.loginfo(f"[EKF Global Init] GPS recovered. EKF drift={drift:.2f}m")
            if self.latest_ekf_pose is None or drift > self.recovery_drift_threshold:
                rospy.logwarn(
                    f"[EKF Global Init] Large drift ({drift:.2f}m). Re-initializing."
                )
                self.reset_pose(x, y, now)
            else:
                rospy.loginfo("[EKF Global Init] Drift small. Resuming without reset.")
            self.is_gps_active = True

        # 2. 갑작스런 점프 감지 (spawn/teleport)
        else:
            dist = math.sqrt((x - self.last_gps_x)**2 + (y - self.last_gps_y)**2)
            if dist > self.jump_threshold:
                rospy.logwarn(
                    f"[EKF Global Init] Sudden GPS jump ({dist:.2f}m). Resetting."
                )
                self.reset_pose(x, y, now)

        self.last_gps_x    = x
        self.last_gps_y    = y
        self.last_gps_time = now

    # ── 초기화 시도 ─────────────────────────────────────────────────
    def _try_initialize(self):
        if self.initialized:
            return

        gps_ok = self.gps_pose_received
        imu_ok = self.imu_received

        # 두 센서 모두 수신 — 즉시 init
        if gps_ok and imu_ok:
            pass
        else:
            # 첫 번째 센서 수신 후 SENSOR_WAIT_TIMEOUT 이상 지났으면 단독 init
            if self._first_sensor_time is None:
                # _first_sensor_time이 아직 설정 안 됐으면 (clock이 0이었을 때)
                # 지금 시계로 재시도
                now = rospy.Time.now()
                if now.to_sec() > 0:
                    self._first_sensor_time = now
                return
            now = rospy.Time.now()
            # use_sim_time=true에서 clock이 아직 0이면 대기
            if now.to_sec() <= 0:
                return
            elapsed = (now - self._first_sensor_time).to_sec()
            if elapsed < self.SENSOR_WAIT_TIMEOUT:
                return  # 아직 대기 중
            if not gps_ok:
                rospy.logwarn(
                    f"[EKF Global Init] GPS not received after {elapsed:.1f}s. "
                    f"Using map origin (0, 0) as fallback."
                )
            if not imu_ok:
                rospy.logwarn(
                    f"[EKF Global Init] IMU not received after {elapsed:.1f}s. "
                    f"Using identity orientation as fallback."
                )

        now = rospy.Time.now()
        self.reset_pose(self.initial_x, self.initial_y, now, _is_init=True)
        self.initialized = True
        rospy.loginfo(
            f"[EKF Global Init] Initialized. "
            f"pos=({self.initial_x:.2f}, {self.initial_y:.2f}) "
            f"yaw={math.degrees(self.latest_yaw):.1f}°. "
            f"Retry loop active until ekf_se_global connects."
        )

    # ── 재전송 타이머 (구독자 연결 전까지) ──────────────────────────
    def _retry_callback(self, event):
        if not self.initialized or self._init_pose_msg is None:
            return
        if self._confirm_count >= self.CONFIRM_SENDS:
            return  # 이미 완료

        n_subs = self.pub_set_pose.get_num_connections()

        # 항상 최신 stamp로 갱신하여 재전송
        self._init_pose_msg.header.stamp = rospy.Time.now()
        self.pub_set_pose.publish(self._init_pose_msg)

        if n_subs > 0:
            self._confirm_count += 1
            rospy.loginfo(
                f"[EKF Global Init] Subscriber connected ({n_subs}). "
                f"Confirm send {self._confirm_count}/{self.CONFIRM_SENDS}."
            )
        else:
            rospy.logwarn_throttle(
                2.0,
                "[EKF Global Init] Waiting for ekf_se_global to subscribe "
                "to /set_pose_global..."
            )

    # ── Pose 발행 (초기화 & 리셋 공용) ──────────────────────────────
    def reset_pose(self, x, y, stamp, _is_init=False):
        now = rospy.Time.now()
        if not _is_init:
            if (now - self.last_reset_time).to_sec() < self.reset_cooldown:
                rospy.logdebug("[EKF Global Init] Reset within cooldown. Ignoring.")
                return

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp    = stamp
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0

        if self.latest_q is not None:
            pose_msg.pose.pose.orientation.x = self.latest_q.x
            pose_msg.pose.pose.orientation.y = self.latest_q.y
            pose_msg.pose.pose.orientation.z = self.latest_q.z
            pose_msg.pose.pose.orientation.w = self.latest_q.w
        else:
            pose_msg.pose.pose.orientation.w = 1.0

        cov = [0.0] * 36
        cov[0]  = 2.0    # X
        cov[7]  = 2.0    # Y
        cov[14] = 999.0  # Z
        cov[21] = 999.0  # Roll
        cov[28] = 999.0  # Pitch
        cov[35] = 0.1    # Yaw
        pose_msg.pose.covariance = cov

        self.pub_set_pose.publish(pose_msg)
        self.last_reset_time = now

        if _is_init:
            # 재전송 타이머용으로 캐시 (IMU 최신 쿼터니언 반영)
            self._init_pose_msg = pose_msg
            self._confirm_count = 0

        rospy.loginfo(
            f"[EKF Global Init] Pose published → "
            f"({x:.2f}, {y:.2f}), yaw={math.degrees(self.latest_yaw):.1f}°"
        )

    # ── GPS 타임아웃 감시 ────────────────────────────────────────────
    def check_gps_timeout(self, event):
        if not self.initialized:
            return
        if self.is_gps_active:
            now     = rospy.Time.now()
            elapsed = (now - self.last_gps_time).to_sec()
            if elapsed > self.gps_timeout:
                self.is_gps_active = False
                rospy.logwarn(
                    f"[EKF Global Init] GPS blackout! "
                    f"No /gps_pose for {elapsed:.1f}s."
                )


if __name__ == '__main__':
    try:
        EKFGlobalInitializer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
