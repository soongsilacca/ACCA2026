#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import CtrlCmd

def morai_control_publisher():
    # 1. 노드 초기화
    rospy.init_node('morai_ctrl_cmd_publisher', anonymous=True)
    
    # 2. 퍼블리셔 생성 (/ctrl_cmd 토픽은 MORAI 시뮬레이터의 기본 수신 토픽입니다)
    ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
    
    # 루프 주기 설정 (예: 20Hz - MORAI 시뮬레이터 제어 주기에 맞추는 것이 좋습니다)
    rate = rospy.Rate(20) 
    
    rospy.loginfo("MORAI CtrlCmd Publisher Started.")

    # 3. 메시지 객체 생성
    ctrl_msg = CtrlCmd()
    
    while not rospy.is_shutdown():
        
        ctrl_msg.longlCmdType = 1 
        ctrl_msg.front_steer = 0.0 
        ctrl_msg.accel = 5  # 목표 속도 20

        
        ctrl_msg.brake = 0  # 전진(D)

        # (참고) longCmdType이 3(페달 제어)일 때는 아래 값들을 사용합니다.
        # ctrl_msg.accel = 0.3   # 0.0 ~ 1.0 (30% 밟음)
        # ctrl_msg.brake = 0.0   # 0.0 ~ 1.0
        
        # 4. 메시지 퍼블리시
        ctrl_pub.publish(ctrl_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        morai_control_publisher()
    except rospy.ROSInterruptException:
        pass