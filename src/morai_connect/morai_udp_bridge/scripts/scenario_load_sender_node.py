#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scenario_load_sender_node.py

ROS node / standalone script to send ScenarioLoad commands to MORAI Simulator via UDP (port 9095).
"""

import sys
from pathlib import Path

# ``lib`` lives in ``morai_connect/lib``.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rospy
from std_msgs.msg import String, Empty
from lib.network.UDP import Sender
from lib.define.ScenarioLoad import SetScenarioLoad


class MoraiScenarioLoadSender:
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node('morai_scenario_load_sender', anonymous=False)

        # Configurable Parameters
        self.ip = rospy.get_param('~ip', '127.0.0.1')
        self.port = rospy.get_param('~port', 9095)
        
        # Default scenario file path or scenario name
        default_file = '/home/acca/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/2026_molit_comp_sample_scene.json'
        self.scenario_file = rospy.get_param('~scenario_file', default_file)
        
        self.delete_all = rospy.get_param('~delete_all', False)
        # Keep the MORAI Network Settings configured in the simulator.  Once
        # the user sets the UDP ports, subsequent scenario reloads must not
        # overwrite them with scenario-file connection data.
        self.network = rospy.get_param('~network', False)
        self.ego = rospy.get_param('~ego', True)
        self.npc = rospy.get_param('~npc', True)
        self.pedestrian = rospy.get_param('~pedestrian', True)
        self.object = rospy.get_param('~object', True)
        # A training scenario must resume immediately after loading.  Setting
        # this true leaves MORAI paused, so no GT/ObjectInfo arrives and the
        # controller correctly remains in safe stop.
        self.pause = rospy.get_param('~pause', False)
        
        self.load_on_start = rospy.get_param('~load_on_start', True)

        self.sender = Sender(self.ip, self.port)

        # Subscribers for triggering scenario load via ROS topics
        self.name_sub = rospy.Subscriber('/morai/scenario_load', String, self.scenario_load_name_callback)
        self.trigger_sub = rospy.Subscriber('/morai/scenario_load_trigger', Empty, self.scenario_load_trigger_callback)

        rospy.loginfo(f"[ScenarioLoad] Initialized node (Target UDP {self.ip}:{self.port})")

    def extract_scenario_name(self, path_or_name: str) -> str:
        """Extract stem/name without directory path and without extension."""
        name = Path(path_or_name).stem
        return name

    def send_scenario_load(self, scenario_input: str = None):
        if scenario_input is None or len(str(scenario_input).strip()) == 0:
            scenario_input = self.scenario_file

        scenario_name = self.extract_scenario_name(scenario_input)
        
        data = SetScenarioLoad()
        data.filename = scenario_name.ljust(30).encode('utf-8')[:30]
        data.delete_all = self.delete_all
        data.network = self.network
        data.ego = self.ego
        data.npc = self.npc
        data.pedestrian = self.pedestrian
        data.object = self.object
        data.pause = self.pause

        self.sender.send(data)
        rospy.loginfo(f"[ScenarioLoad] Sent load command for '{scenario_name}' to UDP {self.ip}:{self.port}")

    def scenario_load_name_callback(self, msg: String):
        self.send_scenario_load(msg.data)

    def scenario_load_trigger_callback(self, msg: Empty):
        self.send_scenario_load(self.scenario_file)

    def run(self):
        if self.load_on_start:
            rospy.sleep(0.3)
            self.send_scenario_load(self.scenario_file)
            
        rospy.spin()


def main():
    try:
        node = MoraiScenarioLoadSender()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
