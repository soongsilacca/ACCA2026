import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import numpy as np
import time
from collections import deque, Counter

from stanley import Stanley
from erp42_msgs.msg import ControlMessage, SerialFeedBack  # (feedback 미사용이어도 유지)


# ---------------------------
# Speed helper
# ---------------------------
class SpeedSupporter:
    def __init__(self, node: Node):
        self.he_gain = node.declare_parameter("/speed_supporter/he_gain_traffic", 40.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain_traffic", 20.0).value
        self.he_thr  = node.declare_parameter("/speed_supporter/he_thr_traffic", 0.05).value
        self.ce_thr  = node.declare_parameter("/speed_supporter/ce_thr_traffic", 0.001).value

    def _lin(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self, value, hdr, ctr, min_value, max_value):
        hdr_term = self._lin(abs(hdr), -self.he_gain, self.he_thr)
        ctr_term = self._lin(abs(ctr), -self.ce_gain, self.ce_thr)
        return np.clip(value + hdr_term + ctr_term, min_value, max_value)


# ---------------------------
# PI controller
# ---------------------------
class PID:
    def __init__(self, node: Node):
        self.node   = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain_traffic", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain_traffic", 0.85).value

        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0

        now = node.get_clock().now().seconds_nanoseconds()
        self.current = now[0] + now[1] / 1e9
        self.last    = self.current

    def PIDControl(self, speed, desired_value, vmin, vmax):
        now = self.node.get_clock().now().seconds_nanoseconds()
        self.current = now[0] + now[1] / 1e9
        dt = max(self.current - self.last, 1e-3)  # dt 보호
        self.last = self.current

        err = desired_value - speed
        self.p_err = err
        self.i_err += self.p_err * dt * (0.0 if speed == 0 else 1.0)

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(self.speed, vmin, vmax))


# ---------------------------
# Traffic light controller
# ---------------------------
class Trafficlight:
    def __init__(self, node: Node):
        self.node = node

        self.st  = Stanley()
        self.pid = PID(node)
        self.ss  = SpeedSupporter(node)

        self.target_idx = 0

        # ── 신호 판단 관련 파라미터 ────────────────────────────────────────
        self.signal_window = deque(
            maxlen=int(node.declare_parameter("/traffic/vote_window", 3).value)
        )
        self.vote_k  = int(node.declare_parameter("/traffic/vote_k", 2).value)  # 1Hz용 과반
        self.vote_hold_s = float(node.declare_parameter("/traffic/vote_hold_s", 0.3).value)
        self.signal_timeout_s = float(node.declare_parameter("/traffic/signal_timeout_s", 3.0).value)
        # 창 크기보다 큰 k는 의미 없음 → 런타임 보정
        self.vote_k = min(self.vote_k, self.signal_window.maxlen)

        self.stable_signal = None        # 확정 신호: 'red'/'yellow'/'green'/None
        self.prev_stable_signal = None
        self.last_stable_change_ts = 0.0
        self.last_signal_time = 0.0      # 유효 신호(빨/노/초) 수신 시각

        # 🔒 빨간불 래치: 초록이 뜨기 전까지 None이어도 정지 유지
        self.must_wait_green = False
        self.red_hold_start_ts = 0.0
        self.red_hold_timeout_s = float(node.declare_parameter("/traffic/red_hold_timeout_s", 60.0).value)

        self.mission_finish = False
        self.start_code = True

        self.target_speed = 8
        self.speed = 0
        self.estop = 0

        self.control_pub = self.node.create_publisher(ControlMessage, "cmd_msg", 10)

        # ✅ YOLO 결과: std_msgs/String("red"/"yellow"/"green"/"none")
        self.traffic_light_sub = self.node.create_subscription(
            String, "/traffic_light_signal", self.signal_callback, 10
        )

        # 인덱스 기반 거리 임계치(맵 해상도 맞춰 조정)
        self.brake_distance_idx = int(node.declare_parameter("/traffic/brake_distance_idx", 30).value)
        self.stop_near_idx      = int(node.declare_parameter("/traffic/stop_near_idx", 3).value)
        self.yellow_stop_idx    = int(node.declare_parameter("/traffic/yellow_stop_idx", 3).value)
    
    def cacluate_brake(
        self, adapted_speed
    ):  # brake 값 정하는 알고리즘 좀 더 정교하게 생각
        if self.odometry.v * 3.6 >= adapted_speed:
            brake = (abs(self.odometry.v * 3.6 - adapted_speed) / 20.0) * 200
            brake = np.clip(brake, 0, 200)
        else:
            brake = 0
        return brake
    
    # ── 라벨 정규화 ─────────────────────────────────────────────────────────
    def _normalize_label(self, name: str):
        if not name:
            return None
        s = name.lower().strip()
        if s == "none":
            return None
        if "red" in s or "stop" in s:
            return "red"
        if "yellow" in s or "amber" in s:
            return "yellow"
        if "green" in s or "go" in s:
            return "green"
        return None

    # ── 다수결 + 타임아웃으로 안정 신호 갱신 ──────────────────────────────
    def _update_stable_signal(self):
        now = time.time()

        # 최근 유효 신호가 일정 시간 없으면 해제(None)
        if self.last_signal_time > 0 and (now - self.last_signal_time) > self.signal_timeout_s:
            if self.stable_signal is not None:
                self.prev_stable_signal = self.stable_signal
                self.stable_signal = None
                self.last_stable_change_ts = now
            # 잡음 축적 방지
            self.signal_window.clear()
            return

        valid = [x for x in self.signal_window if x in ("red", "yellow", "green")]
        if not valid:
            return

        counts = Counter(valid)
        can_switch = (now - self.last_stable_change_ts) >= self.vote_hold_s

        # vote_k 이상인 후보 중 최다 득표 선택
        candidates = [(lab, cnt) for lab, cnt in counts.items() if cnt >= self.vote_k]
        if not candidates:
            return
        candidates.sort(key=lambda x: x[1], reverse=True)
        top_label, _ = candidates[0]

        if self.stable_signal is None:
            self.prev_stable_signal = None
            self.stable_signal = top_label
            self.last_stable_change_ts = now
        else:
            if top_label != self.stable_signal and can_switch:
                self.prev_stable_signal = self.stable_signal
                self.stable_signal = top_label
                self.last_stable_change_ts = now

    # ── String 신호 콜백 ───────────────────────────────────────────────────
    def signal_callback(self, msg: String):
        if not self.start_code:
            return
        lab = self._normalize_label(msg.data)
        now = time.time()
        if lab is not None:
            self.signal_window.append(lab)
            self.last_signal_time = now
        self._update_stable_signal()

    # ── 메인 제어 루프 ────────────────────────────────────────────────────
    def control_traffic_light(self, odometry, path):
        # 콜백이 끊겨도 타임아웃/다수결이 동작하도록 주기적으로 갱신
        self._update_stable_signal()

        # 경로 가드
        npts = len(path.cx)
        if npts == 0:
            msg = ControlMessage()
            msg.steer = 0
            msg.speed = 0
            msg.gear  = 2
            msg.estop = 1
            self.node.get_logger().error("[traffic] empty path → E-Stop")
            return msg, True

        # Stanley 0.5 0.24 bs control stanley 와 paramter 동일 
        steer, idx, hdr, ctr = self.st.stanley_control(
            odometry, path.cx, path.cy, path.cyaw, h_gain=0.5, c_gain=0.24
        )

        # 인덱스/남은 거리
        self.target_idx = max(0, min(int(idx), npts - 1))
        stop_idx = npts - 1
        remaining_idx = max(stop_idx - self.target_idx, 0)

        # 이미 지나침 → 완료
        if remaining_idx <= 0:
            msg = ControlMessage()
            msg.steer = int(np.clip(math.degrees((-1) * steer) * 1e3, -200.0 * 1e3, 200.0 * 1e3))
            msg.speed = 0
            msg.gear  = 2
            msg.estop = 0
            self.reset_traffic_light()
            return msg, True

        sig = self.stable_signal  # 'red'/'yellow'/'green'/None

        # ── 신호별 로직 ─────────────────────────────────────────────────
        if sig == "red":
            # 🔒 래치 ON
            if not self.must_wait_green:
                self.must_wait_green = True
                self.red_hold_start_ts = time.time()

            if remaining_idx <= self.brake_distance_idx:
                self.speed = 0
                self.estop = 1
                self.node.get_logger().info("빨간불: 정지")
            else:
                self.target_speed = int(np.clip((remaining_idx / max(1, npts)) * 15, 6, 8))
                adapted = self.ss.adaptSpeed(self.target_speed, hdr, ctr, 6, 8)
                # brake = self.cacluate_brake(adapted_speed) 속도 올릴시 고려

                self.speed = self.pid.PIDControl(odometry.v * 3.6, adapted, 6, 8)
                self.estop = 0

            if remaining_idx <= self.stop_near_idx:
                self.mission_finish = True

        elif sig == "yellow":
            # 근접 정지 구간이면 🔒 래치 ON
            if remaining_idx <= self.yellow_stop_idx and not self.must_wait_green:
                self.must_wait_green = True
                self.red_hold_start_ts = time.time()

            if remaining_idx <= self.yellow_stop_idx:
                self.speed = 0
                self.estop = 1
                self.node.get_logger().info("노란불 근접: 정지")
            else:
                self.target_speed = 8
                adapted = self.ss.adaptSpeed(self.target_speed, hdr, ctr, 8, 10)
                # brake = self.cacluate_brake(adapted_speed) 속도 올릴시 고려
                self.speed = self.pid.PIDControl(odometry.v * 3.6, adapted, 8, 10)
                self.estop = 0

            if remaining_idx <= self.stop_near_idx:
                self.mission_finish = True

        elif sig == "green":
            # 🔓 초록이면 래치 해제 후 진행
            self.must_wait_green = False
            self.target_speed = 10 # 20 고려
            adapted = self.ss.adaptSpeed(self.target_speed, hdr, ctr, 8, 10)
            self.speed = self.pid.PIDControl(odometry.v * 3.6, adapted, 8, 10)
            self.estop = 0
            self.node.get_logger().info("초록불: 주행 유지")
            if remaining_idx <= self.stop_near_idx:
                self.mission_finish = True

        else:  # sig is None (무신호)
            now = time.time()
            if self.must_wait_green and (now - self.red_hold_start_ts) < self.red_hold_timeout_s:
                # 🔒 초록 뜨기 전까지 정지 유지
                self.speed = 0
                self.estop = 1
                self.node.get_logger().info("신호없음: 빨간불 래치 유지 → 정지")
            else:
                # 래치가 없거나 너무 오래 기다렸을 때만 보수적 주행
                self.target_speed = 10
                adapted = self.ss.adaptSpeed(self.target_speed, hdr, ctr, 8, 10)
                # brake = self.cacluate_brake(adapted_speed) 속도 올릴시 고려\
                self.speed = self.pid.PIDControl(odometry.v * 3.6, adapted, 8, 10)
                self.estop = 0
                self.node.get_logger().info("신호없음: 기본 주행")
            if remaining_idx <= self.stop_near_idx:
                self.mission_finish = True

        # 메시지
        msg = ControlMessage()
        msg.steer = int(np.clip(math.degrees(-steer) * 1e3, -200.0 * 1e3, 200.0 * 1e3))  # 하드웨어에 따라 *10 필요
        msg.speed = int(self.speed) * 10                                # km/h 스케일 *10 가정
        msg.gear  = 2
        msg.estop = self.estop

        # 종료 조건
        if remaining_idx <= 1 or self.mission_finish:
            self.reset_traffic_light()
            return msg, True

        # (선택) 디버그
        # self.node.get_logger().info(f"[traffic] stable={self.stable_signal} latch={self.must_wait_green} rem={remaining_idx}")

        return msg, False

    def reset_traffic_light(self):
        self.signal_window.clear()
        self.stable_signal = None
        self.prev_stable_signal = None
        self.last_stable_change_ts = 0.0
        self.last_signal_time = 0.0
        self.mission_finish = False
        self.start_code = True

        # 🔓 래치 해제
        self.must_wait_green = False
        self.red_hold_start_ts = 0.0
