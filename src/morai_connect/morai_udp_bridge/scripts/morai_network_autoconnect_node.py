#!/usr/bin/env python3
"""Operate MORAI's Network Settings Connect button on a maximized X11 UI."""

import ctypes
import math
import re
import subprocess
import threading
import time

import rospy
from morai_msgs.msg import CtrlCmd, ObjectStatusList
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class XClientMessageData(ctypes.Union):
    _fields_ = [("b", ctypes.c_char * 20),
                ("s", ctypes.c_short * 10),
                ("l", ctypes.c_long * 5)]


class XClientMessageEvent(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int),
                ("serial", ctypes.c_ulong),
                ("send_event", ctypes.c_int),
                ("display", ctypes.c_void_p),
                ("window", ctypes.c_ulong),
                ("message_type", ctypes.c_ulong),
                ("format", ctypes.c_int),
                ("data", XClientMessageData)]


class XEvent(ctypes.Union):
    _fields_ = [("type", ctypes.c_int),
                ("xclient", XClientMessageEvent),
                ("pad", ctypes.c_long * 24)]


class MoraiNetworkAutoConnect:
    def __init__(self):
        rospy.init_node("morai_network_autoconnect")
        self.delay = float(rospy.get_param("~delay_sec", 2.5))
        self.load_disconnect_timeout = float(
            rospy.get_param("~load_disconnect_timeout_sec", 8.0))
        self.load_disconnect_stale = float(
            rospy.get_param("~load_disconnect_stale_sec", 1.0))
        self.post_load_settle = float(
            rospy.get_param("~post_load_settle_sec", 1.0))
        self.stream_freshness = float(
            rospy.get_param("~stream_freshness_sec", 1.5))
        self.stream_stability = float(
            rospy.get_param("~stream_stability_sec", 3.0))
        self.connect_confirmation_timeout = float(
            rospy.get_param("~connect_confirmation_timeout_sec", 12.0))
        self.startup_delay = float(rospy.get_param("~startup_delay_sec", 0.8))
        self.retry_sec = float(rospy.get_param("~retry_sec", 4.0))
        self.max_attempts = int(rospy.get_param("~max_attempts", 3))
        self.connect_x_ratio = float(rospy.get_param("~connect_x_ratio", 0.659))
        self.connect_y_ratio = float(rospy.get_param("~connect_y_ratio", 0.768))
        self.control_stall_sec = float(
            rospy.get_param("~control_stall_sec", 5.0))
        self.control_retry_cooldown_sec = float(
            rospy.get_param("~control_retry_cooldown_sec", 12.0))
        self.minimum_drive_accel = float(
            rospy.get_param("~minimum_drive_accel", 0.1))
        self.minimum_drive_velocity = float(
            rospy.get_param("~minimum_drive_velocity", 1.0))
        self.last_gt = rospy.Time(0)
        self.last_gt_wall = 0.0
        self.last_objects_wall = 0.0
        self.last_cmd = rospy.Time(0)
        self.drive_command_since = None
        self.last_motion = rospy.Time.now()
        self.last_pose = None
        self.last_forced_retry = rospy.Time(0)
        self.episode_loaded = rospy.Time(0)
        self.connect_lock = threading.Lock()
        self.simulator_was_down = False
        self.recovery_pub = rospy.Publisher(
            "/morai/simulator_recovery_request", String,
            queue_size=1, latch=False)
        self.episode_restart_pub = rospy.Publisher(
            "/morai/episode_restart_request", String,
            queue_size=1, latch=False)
        self.generation = 0
        rospy.Subscriber("/morai/episode_status", String,
                         self._episode, queue_size=1)
        rospy.Subscriber("/morai/simulator_status", String,
                         self._simulator_status, queue_size=1)
        rospy.Subscriber("/teacher/ground_truth_state", Odometry,
                         self._gt, queue_size=1)
        rospy.Subscriber("/Object_topic", ObjectStatusList,
                         self._objects, queue_size=1)
        rospy.Subscriber("/ctrl_cmd", CtrlCmd, self._ctrl_cmd, queue_size=1)
        rospy.Timer(rospy.Duration(0.5), self._control_watchdog)
        rospy.loginfo("MORAI maximized-window Network auto-connect ready")
        # Bootstrap is required because ScenarioLoad itself is a MORAI Network
        # subscriber. Connect once before the first load packet is sent.
        threading.Thread(target=self._startup_connect, daemon=True).start()

    def _simulator_status(self, msg):
        state = msg.data.strip().upper()
        if state.startswith("DOWN"):
            self.simulator_was_down = True
            # Cancel any Connect workflow that belonged to the dead process.
            self.generation += 1
            return
        if not state.startswith("READY") or not self.simulator_was_down:
            return
        self.simulator_was_down = False
        self.generation += 1
        generation = self.generation
        rospy.loginfo("MORAI restarted; bootstrapping Network before "
                      "ScenarioLoad")
        threading.Thread(target=self._restart_connect,
                         args=(generation,), daemon=True).start()

    def _restart_connect(self, generation):
        """Restore UDP first; ScenarioLoad cannot be sent without this link."""
        if not self._wait_generation(generation, self.startup_delay):
            return
        for attempt in range(1, self.max_attempts + 1):
            if rospy.is_shutdown() or generation != self.generation:
                return
            if self._wait_for_stable_streams(
                    generation, self.stream_stability + 0.5):
                rospy.loginfo("MORAI restart Network already stable "
                              "(GT/ObjectInfo)")
                return
            try:
                self._serialized_click_connect()
                rospy.loginfo("MORAI restart bootstrap Network Connect "
                              "requested (%d/%d)", attempt,
                              self.max_attempts)
            except Exception as exc:
                rospy.logerr("MORAI restart bootstrap Connect failed: %s", exc)
                if "Simulator window not found" in str(exc):
                    self.recovery_pub.publish(String("network_window_missing"))
                return
            if self._wait_for_stable_streams(
                    generation, self.connect_confirmation_timeout):
                rospy.loginfo("MORAI restart Network confirmed by stable "
                              "GT/ObjectInfo; ScenarioLoad may proceed")
                return
        rospy.logwarn("MORAI restart Network did not restore stable "
                      "GT/ObjectInfo after %d attempts", self.max_attempts)
        self.recovery_pub.publish(String("restart_network_unavailable"))

    def _startup_connect(self):
        time.sleep(self.startup_delay)
        if rospy.is_shutdown():
            return
        # MORAI may already be connected when this launch starts. Its button
        # is a Connect/Disconnect toggle, so clicking blindly would tear down
        # a healthy session. Fresh GT packets are the authoritative connection
        # check used by the rest of this node as well.
        if self._streams_fresh():
            rospy.loginfo("MORAI Network already connected at startup "
                          "(GT/ObjectInfo active)")
            return
        try:
            self._serialized_click_connect()
            rospy.loginfo("MORAI bootstrap Network Connect requested")
        except Exception as exc:
            rospy.logerr("MORAI bootstrap Network auto-connect failed: %s", exc)
            if "Simulator window not found" in str(exc):
                self.recovery_pub.publish(String("network_window_missing"))

    def _gt(self, msg):
        now = rospy.Time.now()
        self.last_gt = now
        self.last_gt_wall = time.monotonic()
        point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        speed = math.hypot(msg.twist.twist.linear.x,
                           msg.twist.twist.linear.y)
        if (self.last_pose is None or speed > 0.3 or
                math.hypot(point[0] - self.last_pose[0],
                           point[1] - self.last_pose[1]) > 0.25):
            self.last_motion = now
            self.last_pose = point

    def _objects(self, _msg):
        self.last_objects_wall = time.monotonic()

    def _streams_fresh(self):
        now = time.monotonic()
        return (self.last_gt_wall > 0.0 and
                self.last_objects_wall > 0.0 and
                now - self.last_gt_wall <= self.stream_freshness and
                now - self.last_objects_wall <= self.stream_freshness)

    def _wait_for_stable_streams(self, generation, timeout):
        """Require both UDP status streams to remain fresh continuously."""
        deadline = time.monotonic() + timeout
        stable_since = None
        while not rospy.is_shutdown() and generation == self.generation:
            now = time.monotonic()
            if self._streams_fresh():
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= self.stream_stability:
                    return True
            else:
                stable_since = None
            if now >= deadline:
                return False
            time.sleep(0.1)
        return False

    def _wait_generation(self, generation, duration):
        deadline = time.monotonic() + duration
        while not rospy.is_shutdown() and generation == self.generation:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return True
            time.sleep(min(0.1, remaining))
        return False

    def _ctrl_cmd(self, msg):
        now = rospy.Time.now()
        self.last_cmd = now
        drive = ((int(msg.cmd_type) == 1 and
                  float(msg.accel) >= self.minimum_drive_accel and
                  float(msg.brake) < 0.1) or
                 (int(msg.cmd_type) == 2 and
                  float(msg.velocity) >= self.minimum_drive_velocity and
                  float(msg.brake) < 0.1))
        if drive:
            if self.drive_command_since is None:
                self.drive_command_since = now
        else:
            self.drive_command_since = None

    def _control_watchdog(self, _event):
        now = rospy.Time.now()
        if (self.drive_command_since is None or
                (now - self.last_cmd).to_sec() > 1.0 or
                (now - self.drive_command_since).to_sec() < self.control_stall_sec or
                (now - self.last_motion).to_sec() < self.control_stall_sec or
                (now - self.episode_loaded).to_sec() < self.control_stall_sec or
                (now - self.last_forced_retry).to_sec() < self.control_retry_cooldown_sec):
            return
        self.last_forced_retry = now
        self.drive_command_since = now
        self.last_motion = now
        rospy.logerr("Drive ctrl_cmd is active but ego has been stationary "
                     "for %.1fs; requesting a fresh scenario",
                     self.control_stall_sec)
        self.episode_restart_pub.publish(String("control_stall"))

    def _episode(self, msg):
        if "LOAD_SENT" in msg.data:
            # ScenarioLoad can temporarily drop every UDP stream even when
            # load_network_connection_data is false. Waiting for LOADED here
            # deadlocks: the episode manager requires fresh GT/ObjectInfo to
            # acknowledge the load before it can publish LOADED.
            self.generation += 1
            self.episode_loaded = rospy.Time.now()
            self.drive_command_since = None
            generation = self.generation
            threading.Thread(target=self._post_load_reconnect,
                             args=(generation,),
                             daemon=True).start()
            rospy.loginfo("Scenario LOAD_SENT; waiting for scenario load before "
                          "Network reconnect")
            return
        if "LOADED" not in msg.data:
            return
        # LOADED now means GT and ObjectInfo are already verified. Do not
        # start another blind Connect-toggle cycle.
        self.episode_loaded = rospy.Time.now()
        self.drive_command_since = None
        self.last_motion = rospy.Time.now()

    def _post_load_reconnect(self, generation):
        """Reconnect only after ScenarioLoad has dropped and rebuilt the scene."""
        # A ScenarioLoad packet is only a request.  Clicking Connect a fixed
        # short time after LOAD_SENT can hit the UI while Unity is still
        # loading.  First wait until the old GT stream has actually stopped,
        # then give the newly loaded scene a settle window before touching the
        # Connect/Disconnect toggle.
        disconnect_deadline = time.monotonic() + self.load_disconnect_timeout
        saw_disconnect = False
        while not rospy.is_shutdown() and generation == self.generation:
            if (self.last_gt_wall <= 0.0 or
                    time.monotonic() - self.last_gt_wall >=
                    self.load_disconnect_stale):
                saw_disconnect = True
                rospy.loginfo("Scenario load disconnected the old Network session")
                break
            if time.monotonic() >= disconnect_deadline:
                rospy.logwarn("GT did not visibly drop after ScenarioLoad; "
                              "using post-load settle delay before reconnect")
                break
            time.sleep(0.2)
        if rospy.is_shutdown() or generation != self.generation:
            return
        if not self._wait_generation(generation, self.post_load_settle):
            return
        rospy.loginfo("Scenario load settle complete%s; checking Network",
                      " after GT disconnect" if saw_disconnect else "")
        for attempt in range(1, self.max_attempts + 1):
            if rospy.is_shutdown() or generation != self.generation:
                return
            # Connect is a toggle. Never click it when both receive streams
            # have already proven stable after the scene reload.
            if self._wait_for_stable_streams(
                    generation, self.stream_stability + 0.5):
                rospy.loginfo("MORAI post-load Network already stable "
                              "(GT/ObjectInfo)")
                return
            try:
                self._serialized_click_connect()
                rospy.loginfo("MORAI post-ScenarioLoad Network Connect "
                              "requested (%d/%d)", attempt,
                              self.max_attempts)
            except Exception as exc:
                rospy.logerr("MORAI post-load Network reconnect failed: %s",
                             exc)
                if "Simulator window not found" in str(exc):
                    self.recovery_pub.publish(String("network_window_missing"))
                return
            if self._wait_for_stable_streams(
                    generation, self.connect_confirmation_timeout):
                rospy.loginfo("MORAI post-load Network reconnect confirmed by "
                              "stable GT/ObjectInfo")
                return
        rospy.logwarn("MORAI post-load Network reconnect did not restore stable "
                      "GT/ObjectInfo after %d attempts", self.max_attempts)

    def _attempt_loop(self, generation, initial_delay=None):
        time.sleep(self.delay if initial_delay is None else initial_delay)
        for attempt in range(1, self.max_attempts + 1):
            if rospy.is_shutdown() or generation != self.generation:
                return
            if self._streams_fresh():
                rospy.loginfo("MORAI Network already connected "
                              "(GT/ObjectInfo active)")
                return
            try:
                self._serialized_click_connect()
                rospy.loginfo("MORAI Network Connect requested (%d/%d)",
                              attempt, self.max_attempts)
            except Exception as exc:
                rospy.logerr("MORAI Network auto-connect failed: %s", exc)
                if "Simulator window not found" in str(exc):
                    self.recovery_pub.publish(String("network_window_missing"))
                return
            time.sleep(self.retry_sec)
        if not self._streams_fresh():
            rospy.logwarn("MORAI Network Connect did not restore GT/ObjectInfo "
                          "after %d attempts",
                          self.max_attempts)

    def _serialized_click_connect(self):
        with self.connect_lock:
            self._click_connect()

    def _click_connect(self):
        x11 = ctypes.cdll.LoadLibrary("libX11.so.6")
        xtst = ctypes.cdll.LoadLibrary("libXtst.so.6")
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XDefaultScreen.argtypes = [ctypes.c_void_p]
        x11.XDisplayWidth.argtypes = [ctypes.c_void_p, ctypes.c_int]
        x11.XDisplayHeight.argtypes = [ctypes.c_void_p, ctypes.c_int]
        x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        x11.XDefaultRootWindow.restype = ctypes.c_ulong
        x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p,
                                    ctypes.c_int]
        x11.XInternAtom.restype = ctypes.c_ulong
        x11.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong,
                                   ctypes.c_int, ctypes.c_long,
                                   ctypes.POINTER(XEvent)]
        x11.XSendEvent.restype = ctypes.c_int
        x11.XMapRaised.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        x11.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        x11.XSetInputFocus.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
        x11.XFlush.argtypes = [ctypes.c_void_p]
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        xtst.XTestFakeMotionEvent.argtypes = [
            ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
            ctypes.c_int, ctypes.c_ulong]
        xtst.XTestFakeButtonEvent.argtypes = [
            ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
        xtst.XTestFakeKeyEvent.argtypes = [
            ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
        display = x11.XOpenDisplay(None)
        if not display:
            raise RuntimeError("cannot open X11 display")
        try:
            screen = x11.XDefaultScreen(display)
            width = x11.XDisplayWidth(display, screen)
            height = x11.XDisplayHeight(display, screen)

            tree = subprocess.check_output(
                ["xwininfo", "-root", "-tree"], text=True)
            match = re.search(r'^\s*(0x[0-9a-fA-F]+)\s+"Simulator"',
                              tree, re.MULTILINE)
            if not match:
                raise RuntimeError("maximized MORAI Simulator window not found")
            simulator_window = int(match.group(1), 16)

            # Ask the EWMH window manager to make MORAI the real active window.
            # Merely moving/clicking the pointer or calling XSetInputFocus does
            # not reliably activate a Unity window under common Linux desktops.
            root = x11.XDefaultRootWindow(display)
            active_atom = x11.XInternAtom(display, b"_NET_ACTIVE_WINDOW", 0)
            event = XEvent()
            event.xclient.type = 33  # ClientMessage
            event.xclient.display = display
            event.xclient.window = simulator_window
            event.xclient.message_type = active_atom
            event.xclient.format = 32
            event.xclient.data.l[0] = 2  # source: pager/application
            event.xclient.data.l[1] = 0  # CurrentTime
            event.xclient.data.l[2] = 0
            if not x11.XSendEvent(display, root, False,
                                  (1 << 20) | (1 << 19),
                                  ctypes.byref(event)):
                raise RuntimeError("window manager rejected MORAI activation")
            x11.XMapRaised(display, simulator_window)
            x11.XRaiseWindow(display, simulator_window)
            x11.XSetInputFocus(display, simulator_window, 1, 0)
            x11.XFlush(display)
            time.sleep(0.6)

            active = subprocess.check_output(
                ["xprop", "-root", "_NET_ACTIVE_WINDOW"], text=True)
            active_match = re.search(r"0x[0-9a-fA-F]+", active)
            if (not active_match or
                    int(active_match.group(0), 16) != simulator_window):
                raise RuntimeError(
                    "MORAI activation was not acknowledged by window manager")

            f4 = x11.XKeysymToKeycode(display, 0xFFC1)
            xtst.XTestFakeKeyEvent(display, f4, True, 0)
            xtst.XTestFakeKeyEvent(display, f4, False, 0)
            x11.XFlush(display)
            time.sleep(0.8)

            x = int(width * self.connect_x_ratio)
            y = int(height * self.connect_y_ratio)
            xtst.XTestFakeMotionEvent(display, screen, x, y, 0)
            xtst.XTestFakeButtonEvent(display, 1, True, 0)
            xtst.XTestFakeButtonEvent(display, 1, False, 0)
            x11.XFlush(display)
            time.sleep(0.8)

            escape = x11.XKeysymToKeycode(display, 0xFF1B)
            xtst.XTestFakeKeyEvent(display, escape, True, 0)
            xtst.XTestFakeKeyEvent(display, escape, False, 0)
            x11.XFlush(display)
        finally:
            x11.XCloseDisplay(display)


if __name__ == "__main__":
    MoraiNetworkAutoConnect()
    rospy.spin()
