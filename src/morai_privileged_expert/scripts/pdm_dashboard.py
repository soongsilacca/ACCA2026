#!/usr/bin/env python3
import rospy
import curses
import threading
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from morai_msgs.msg import CtrlCmd
import math

class PDMDashboard:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.lock = threading.Lock()
        
        self.episode_status = "Waiting..."
        self.sim_status = "Waiting..."
        self.behavior = "Waiting..."
        self.target_speed = 0.0
        self.current_speed = 0.0
        self.ctrl_accel = 0.0
        self.ctrl_brake = 0.0
        self.ctrl_steer = 0.0
        
        rospy.init_node('pdm_dashboard', anonymous=True)
        rospy.Subscriber("/morai/episode_status", String, self.cb_episode)
        rospy.Subscriber("/morai/simulator_status", String, self.cb_sim)
        rospy.Subscriber("/privileged_expert/behavior", String, self.cb_behavior)
        rospy.Subscriber("/privileged_expert/target_velocity", Float32, self.cb_target_vel)
        rospy.Subscriber("/localization/kinematic_state", Odometry, self.cb_odom)
        rospy.Subscriber("/ctrl_cmd", CtrlCmd, self.cb_ctrl)
        
    def cb_episode(self, msg):
        with self.lock: self.episode_status = msg.data
    def cb_sim(self, msg):
        with self.lock: self.sim_status = msg.data
    def cb_behavior(self, msg):
        with self.lock: self.behavior = msg.data
    def cb_target_vel(self, msg):
        with self.lock: self.target_speed = msg.data * 3.6
    def cb_odom(self, msg):
        with self.lock:
            self.current_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y) * 3.6
    def cb_ctrl(self, msg):
        with self.lock:
            self.ctrl_accel = msg.accel
            self.ctrl_brake = msg.brake
            self.ctrl_steer = msg.steer
            
    def run(self):
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        self.stdscr.nodelay(True)
        
        while not rospy.is_shutdown():
            with self.lock:
                self.stdscr.erase()
                
                # Title
                self.stdscr.addstr(0, 0, "=== MORAI PDM & SCENARIO DASHBOARD ===", curses.A_BOLD | curses.A_REVERSE)
                
                # Simulator & Episode State
                self.stdscr.addstr(2, 0, "[[ SYSTEM STATUS ]]", curses.A_BOLD)
                self.stdscr.addstr(3, 2, f"Simulator Status : {self.sim_status}")
                self.stdscr.addstr(4, 2, f"Episode Status   : {self.episode_status}")
                
                # PDM State
                self.stdscr.addstr(6, 0, "[[ PDM EXPERT STATE ]]", curses.A_BOLD)
                self.stdscr.addstr(7, 2, f"Behavior         : {self.behavior}")
                self.stdscr.addstr(8, 2, f"Target Speed     : {self.target_speed:5.1f} km/h")
                self.stdscr.addstr(9, 2, f"Current Speed    : {self.current_speed:5.1f} km/h")
                
                # Control Output
                self.stdscr.addstr(11, 0, "[[ VEHICLE CONTROL ]]", curses.A_BOLD)
                
                # Simple progress bars for control
                accel_bar = "|" + "#" * int(self.ctrl_accel * 20) + " " * (20 - int(self.ctrl_accel * 20)) + "|"
                brake_bar = "|" + "#" * int(self.ctrl_brake * 20) + " " * (20 - int(self.ctrl_brake * 20)) + "|"
                
                # Steer bar is bidirectional
                steer_normalized = max(-1.0, min(1.0, self.ctrl_steer))
                steer_pos = int((steer_normalized + 1.0) * 10)
                steer_bar = "|" + " " * steer_pos + "#" + " " * (20 - steer_pos) + "|"
                
                self.stdscr.addstr(12, 2, f"Accel : {self.ctrl_accel:4.2f}  {accel_bar}")
                self.stdscr.addstr(13, 2, f"Brake : {self.ctrl_brake:4.2f}  {brake_bar}")
                self.stdscr.addstr(14, 2, f"Steer : {self.ctrl_steer:5.2f}  {steer_bar}")
                
                self.stdscr.addstr(17, 0, "Press Ctrl+C to exit.", curses.A_DIM)
                self.stdscr.refresh()
                
            rospy.sleep(0.05)

if __name__ == "__main__":
    def main(stdscr):
        dash = PDMDashboard(stdscr)
        dash.run()
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
