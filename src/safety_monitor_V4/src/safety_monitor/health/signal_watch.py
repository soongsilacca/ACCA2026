"""6.2 센서 신호 감시.

신호마다 기준 주파수 f_i와 허용 결손 주기 수 N_i로 두절 임계 T_i = N_i / f_i 를 둔다.
- UNINIT: 첫 유효 메시지 전. 두절 판정은 하지 않지만 전체 감시는 미준비(정지 유지).
- OK: 정상
- LOST: 마지막 수신 후 T_i 초과. 유효 메시지가 다시 release_time 동안 끊김 없이
        들어오면 해제 (6.1.2절 해제 규칙과 동일한 0.68 s).
시각은 단조 증가 시계(time.monotonic)를 쓴다. 시뮬레이터 시계가 멈춰도 두절을 잡는다.
"""

UNINIT = "UNINIT"
OK = "OK"
LOST = "LOST"


class SignalWatch:
    def __init__(self, name, rate_hz, allowed_missing, release_time):
        if rate_hz <= 0 or allowed_missing <= 0:
            raise ValueError("rate_hz and allowed_missing must be positive")
        self.name = name
        self.timeout = float(allowed_missing) / float(rate_hz)
        self.release_time = float(release_time)
        self.last = None
        self.latched = False
        self.recovery_start = None

    def on_message(self, now, valid=True):
        if not valid:
            return
        if self.latched and self.recovery_start is None:
            self.recovery_start = now
        self.last = now

    def evaluate(self, now):
        if self.last is None:
            return UNINIT
        if now - self.last > self.timeout:
            self.latched = True
            self.recovery_start = None
            return LOST
        if self.latched:
            if self.recovery_start is not None and now - self.recovery_start >= self.release_time:
                self.latched = False
                self.recovery_start = None
                return OK
            return LOST
        return OK


class HealthMonitor:
    def __init__(self, watches):
        self.watches = list(watches)

    def evaluate(self, now):
        """반환: (estop, reason, {name: state})"""
        states = {w.name: w.evaluate(now) for w in self.watches}
        lost = [n for n, s in states.items() if s == LOST]
        uninit = [n for n, s in states.items() if s == UNINIT]
        if lost:
            return True, "lost:" + ",".join(lost), states
        if uninit:
            return True, "not_ready:" + ",".join(uninit), states
        return False, "ok", states
