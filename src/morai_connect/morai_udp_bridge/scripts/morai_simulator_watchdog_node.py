#!/usr/bin/env python3
"""Restart MORAI after an unexpected simulator process exit."""

import os
import re
import signal
import subprocess
import time
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class MoraiSimulatorWatchdog:
    def __init__(self):
        rospy.init_node("morai_simulator_watchdog")
        self.executable = Path(os.path.expanduser(rospy.get_param(
            "~executable",
            "/home/acca/MoraiLauncher_Lin/MoraiLauncher_Lin.x86_64"))).resolve()
        self.restart_delay = float(rospy.get_param("~restart_delay_sec", 3.0))
        self.window_timeout = float(rospy.get_param("~window_timeout_sec", 120.0))
        self.retry_delay = float(rospy.get_param("~retry_delay_sec", 15.0))
        self.window_title = rospy.get_param("~window_title", "Simulator")
        self.recovery_request_grace = float(rospy.get_param(
            "~recovery_request_grace_sec", 5.0))
        self.hang_gt_timeout = float(rospy.get_param(
            "~hang_gt_timeout_sec", 30.0))
        self.last_gt = rospy.Time(0)
        self.ever_had_gt = False
        self.hang_handled = False
        self.scenario_loading = False
        self.status_pub = rospy.Publisher("/morai/simulator_status", String,
                                          queue_size=1, latch=True)
        self.was_running = self._is_running()
        simulator_ready = self.was_running and self._window_exists()
        self.recovering = not simulator_ready
        self.recovery_requested_at = None
        self.recovery_reason = ""
        self.window_missing_since = (time.monotonic()
                                     if self.was_running and not simulator_ready
                                     else None)
        self.child = None
        self.last_attempt = 0.0
        rospy.Subscriber("/teacher/ground_truth_state", Odometry,
                         self._gt, queue_size=1)
        rospy.Subscriber("/morai/simulator_recovery_request", String,
                         self._recovery_request, queue_size=1)
        rospy.Subscriber("/morai/episode_status", String,
                         self._episode_status, queue_size=1)
        if simulator_ready:
            self._publish("READY existing_process")
        else:
            self._publish("DOWN startup_window_missing")
        rospy.Timer(rospy.Duration(1.0), self._tick)
        rospy.loginfo("MORAI process watchdog ready: %s", self.executable)

    def _publish(self, text):
        self.status_pub.publish(String(text))
        rospy.loginfo("MORAI watchdog: %s", text)

    def _process_ids(self):
        target = str(self.executable)
        result = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                if os.path.realpath(str(entry / "exe")) == target:
                    result.append(int(entry.name))
            except (FileNotFoundError, PermissionError, OSError):
                continue
        return result

    def _is_running(self):
        return bool(self._process_ids())

    def _gt(self, _msg):
        self.last_gt = rospy.Time.now()
        self.ever_had_gt = True
        self.hang_handled = False

    def _episode_status(self, msg):
        state = msg.data.upper()
        if "LOAD_SENT" in state:
            # ScenarioLoad normally tears down the UDP endpoints temporarily.
            # A missing GT stream during that interval is not a Unity hang.
            self.scenario_loading = True
        elif "LOADED" in state or "LOAD_FAILED" in state:
            self.scenario_loading = False

    def _recovery_request(self, msg):
        # Do not kill on the ROS callback itself. Give Unity a short grace
        # period to finish creating/renaming its Simulator window.
        self.recovery_requested_at = time.monotonic()
        self.recovery_reason = msg.data.strip() or "external_request"
        rospy.logwarn("MORAI recovery requested: %s", self.recovery_reason)

    def _not_responding_dialog(self):
        try:
            tree = subprocess.check_output(
                ["xwininfo", "-root", "-tree"], text=True,
                stderr=subprocess.DEVNULL, timeout=3.0)
            return bool(re.search(
                r'(is not responding|not responding|응답하지 않)', tree,
                re.IGNORECASE))
        except (subprocess.SubprocessError, OSError):
            return False

    def _terminate_hung_simulator(self, reason):
        if self.hang_handled:
            return
        self.hang_handled = True
        self.was_running = False
        self.recovering = True
        # Never carry the previous process' GT timestamp into its replacement.
        self.ever_had_gt = False
        self.last_gt = rospy.Time(0)
        self.scenario_loading = False
        self.recovery_requested_at = None
        self.last_attempt = time.monotonic()
        self._publish("DOWN %s" % reason)
        pids = self._process_ids()
        rospy.logerr("MORAI is unresponsive; terminating exact simulator "
                     "process(es): %s", pids)
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 5.0
        while self._process_ids() and time.monotonic() < deadline:
            time.sleep(0.1)
        for pid in self._process_ids():
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def _window_exists(self):
        try:
            tree = subprocess.check_output(
                ["xwininfo", "-root", "-tree"], text=True,
                stderr=subprocess.DEVNULL, timeout=3.0)
            return re.search(r'"%s(?:\s|\")' % re.escape(self.window_title),
                             tree) is not None
        except (subprocess.SubprocessError, OSError):
            return False

    def _tick(self, _event):
        running = self._is_running()
        if running:
            window_ready = self._window_exists()
            if window_ready:
                self.window_missing_since = None
            elif self.window_missing_since is None:
                self.window_missing_since = time.monotonic()
            elif (self.recovering and self.window_missing_since is not None and
                  time.monotonic() - self.window_missing_since >=
                  self.window_timeout):
                self._terminate_hung_simulator("simulator_window_timeout")
                return

            # A recovery request is explicit. It must not be silently erased
            # merely because the Unity window still exists.
            if (self.recovery_requested_at is not None and
                    time.monotonic() - self.recovery_requested_at >=
                    self.recovery_request_grace):
                reason = self.recovery_reason
                self._terminate_hung_simulator("requested_%s" % reason)
                return

            if self._not_responding_dialog():
                self._terminate_hung_simulator("not_responding_dialog")
                return

            # A newly launched process must become READY before stale data from
            # the old process can participate in health checks.
            if self.recovering:
                if window_ready:
                    self.was_running = True
                    self.recovering = False
                    self.ever_had_gt = False
                    self.last_gt = rospy.Time(0)
                    self.hang_handled = False
                    self._publish("READY restarted_process")
                return

            gt_stale = (self.ever_had_gt and
                        not self.scenario_loading and
                        self.recovery_requested_at is None and
                        (rospy.Time.now() - self.last_gt).to_sec() >=
                        self.hang_gt_timeout)
            if gt_stale:
                self._terminate_hung_simulator(
                    "gt_stale_%.0fs" % self.hang_gt_timeout)
                return
            return

        if self.was_running:
            self.was_running = False
            self.recovering = True
            self.last_attempt = 0.0
            self.ever_had_gt = False
            self.last_gt = rospy.Time(0)
            self.scenario_loading = False
            self._publish("DOWN process_exited")

        now = time.monotonic()
        if now - self.last_attempt < (self.restart_delay if not self.last_attempt
                                      else self.retry_delay):
            return
        self.last_attempt = now
        if not self.executable.is_file():
            rospy.logerr_throttle(30.0, "MORAI executable not found: %s",
                                  self.executable)
            return
        try:
            # Unity fullscreen makes the GUI coordinates used by the existing
            # network auto-connect node deterministic after recovery.
            self.child = subprocess.Popen(
                [str(self.executable), "-screen-fullscreen", "1"],
                cwd=str(self.executable.parent), start_new_session=True,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.recovering = True
            self.hang_handled = False
            self.ever_had_gt = False
            self.last_gt = rospy.Time(0)
            self.scenario_loading = False
            self.recovery_requested_at = None
            self.window_missing_since = time.monotonic()
            rospy.logwarn("MORAI restart requested (pid=%d)", self.child.pid)
        except OSError as exc:
            rospy.logerr("Could not restart MORAI: %s", exc)


if __name__ == "__main__":
    MoraiSimulatorWatchdog()
    rospy.spin()
