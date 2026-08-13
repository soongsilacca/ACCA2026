#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class DummyPathPublisher:
    def __init__(self):
        rospy.init_node('dummy_path_node', anonymous=True)
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=10)
        self.rate = rospy.Rate(1) # 경로는 정적이므로 1초에 한 번만 뿌려도 충분해

    def run(self):
        while not rospy.is_shutdown():
            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = "map"
            
            # [수정] X축으로 전진하면서 Y축이 부드러운 S자 곡선(사인파)을 그리도록 변경
            # 총 150m 구간, 0.5m 간격으로 총 300개의 웨이포인트를 생성해
            for i in range(300):
                pose = PoseStamped()
                x = i * 0.5
                
                # 💡 수학 공식: y = A * sin(x / B)
                # 3.0: 좌우로 최대 3미터까지 휘어지는 진폭(Amplitude)
                # 10.0: 곡선이 얼마나 완만하게 휘어지는지 결정하는 나사 (클수록 완만해짐)
                y = 3.0 * math.sin(x / 6.0) 
                
                pose.pose.position.x = x
                pose.pose.position.y = y
                path_msg.poses.append(pose)
                
            self.path_pub.publish(path_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        pub = DummyPathPublisher()
        pub.run()
    except rospy.ROSInterruptException:
        pass