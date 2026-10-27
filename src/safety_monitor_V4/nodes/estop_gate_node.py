#!/usr/bin/env python3
"""E-STOP 발행: 두 감시 결과를 OR로 합쳐 제어 명령을 직접 덮어쓴다.

  planner/제어기 -> ~cmd_in (/ctrl_cmd_raw) -> [gate] -> ~cmd_out (/ctrl_cmd) -> MORAI

정지 조건 (하나라도 해당하면 제동 명령 발행)
  - /safety/collision_estop 또는 /safety/sensor_estop 이 true
  - 두 입력 중 하나라도 아직 수신 전이거나 timeout 초과 (받지 못한 것을 false로 보지 않음)
  - 제어 명령(~cmd_in)이 timeout 초과

판정 로직은 두지 않는다. 이 노드가 죽으면 MORAI는 마지막 명령을 유지할 수 있으므로
최소 기능만 둔다.
"""
import threading
import time

import rospy
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import Bool, String


class _Input:
    def __init__(self, name, timeout):
        self.name = name
        self.timeout = timeout
        self.value = None
        self.rx = None

    def set(self, value):
        self.value = bool(value)
        self.rx = time.monotonic()

    def state(self, now):
        if self.rx is None:
            return "not_ready"
        if now - self.rx > self.timeout:
            return "stale"
        return "estop" if self.value else "ok"


class EstopGateNode:
    def __init__(self):
        self.inputs = [
            _Input("collision", float(rospy.get_param("~collision_timeout", 1.5))),
            _Input("sensor", float(rospy.get_param("~sensor_timeout", 1.5))),
        ]
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.2))
        self.require_cmd = bool(rospy.get_param("~require_cmd", True))
        self.brake_value = float(rospy.get_param("~brake", 1.0))
        cmd_in = rospy.get_param("~cmd_in", "/ctrl_cmd_raw")
        cmd_out = rospy.get_param("~cmd_out", "/ctrl_cmd")

        self.last_cmd = None
        self.last_cmd_rx = None
        self.last_reason = None
        self.lock = threading.Lock()

        self.pub_cmd = rospy.Publisher(cmd_out, CtrlCmd, queue_size=1)
        self.pub_estop = rospy.Publisher("/safety/estop", Bool, queue_size=1)
        self.pub_reason = rospy.Publisher("/safety/estop_reason", String, queue_size=1, latch=True)

        rospy.Subscriber("/safety/collision_estop", Bool,
                         lambda m: self.inputs[0].set(m.data), queue_size=1)
        rospy.Subscriber("/safety/sensor_estop", Bool,
                         lambda m: self.inputs[1].set(m.data), queue_size=1)
        rospy.Subscriber(cmd_in, CtrlCmd, self._cmd_cb, queue_size=1)
        rate = float(rospy.get_param("~rate", 50.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self._tick)
        rospy.loginfo("estop gate: %s -> %s", cmd_in, cmd_out)

    def _decide(self, now):
        reasons = []
        for inp in self.inputs:
            st = inp.state(now)
            if st != "ok":
                reasons.append("%s_%s" % (inp.name, st))
        if self.require_cmd:
            if self.last_cmd_rx is None:
                reasons.append("cmd_not_ready")
            elif now - self.last_cmd_rx > self.cmd_timeout:
                reasons.append("cmd_stale")
        return (len(reasons) > 0), (",".join(reasons) if reasons else "ok")

    def _brake_cmd(self):
        cmd = CtrlCmd()
        # Workspace-local morai_msgs/CtrlCmd uses cmd_type/steer (not the
        # longlCmdType/steering fields used by some MORAI message releases).
        cmd.ctrl_mode = self.last_cmd.ctrl_mode if self.last_cmd is not None else 2
        cmd.gear = self.last_cmd.gear if self.last_cmd is not None else 4
        cmd.cmd_type = 1
        cmd.accel = 0.0
        cmd.brake = self.brake_value
        cmd.steer = self.last_cmd.steer if self.last_cmd is not None else 0.0
        return cmd

    def _publish(self, now):
        with self.lock:
            self._publish_locked(now)

    def _publish_locked(self, now):
        estop, reason = self._decide(now)
        self.pub_cmd.publish(self._brake_cmd() if estop else self.last_cmd)
        self.pub_estop.publish(Bool(data=estop))
        if reason != self.last_reason:
            self.pub_reason.publish(String(data=reason))
            if estop:
                rospy.logwarn("E-STOP: %s", reason)
            else:
                rospy.loginfo("E-STOP released")
            self.last_reason = reason

    def _cmd_cb(self, msg):
        self.last_cmd = msg
        self.last_cmd_rx = time.monotonic()
        self._publish(self.last_cmd_rx)     # 정상일 때 지연 없이 전달

    def _tick(self, _evt):
        now = time.monotonic()
        estop, _ = self._decide(now)
        if estop:                           # 정지 중이거나 명령이 끊겼을 때는 주기적으로 제동 명령
            self._publish(now)
        else:
            self.pub_estop.publish(Bool(data=False))


def main():
    rospy.init_node("estop_gate")
    EstopGateNode()
    rospy.spin()


if __name__ == "__main__":
    main()
