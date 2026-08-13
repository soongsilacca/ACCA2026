import numpy as np
import math as m
import rospy
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import Float32

class state:
    """ 차량의 현재 물리 상태를 저장하는 구조체 클래스 """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

class pure_pursuit:
    def __init__(self):
        self.k = 0.4
        self.WB = 3.0       # Wheel Base 
        self.Lfc = 6.0      # 기본 전방 주시 거리
        self.max_steer = 40.0 * m.pi / 180.0

        self.Lf = self.Lfc

        self.rear_x = 0.0
        self.rear_y = 0.0
        self.ti = 0

    def calc_target_index(self, state, cx, cy):
        self.rear_x = state.x
        self.rear_y = state.y
        self.Lf = self.k * state.v + self.Lfc

        distance = [np.hypot(self.rear_x - cx[i], self.rear_y - cy[i])
                    for i in range(len(cx))]
        ti = np.argmin(distance)

        while self.Lf > distance[ti]:
            if ti + 1 >= len(cx):
                break
            ti += 1
        self.ti = ti
        idx = self.ti

        return idx

    def pure_pursuit_control(self, state, cx, cy):

        idx = self.calc_target_index(state, cx, cy)

        rospy.loginfo_throttle(0.1, f'last_idx : {idx}')

        tx = cx[idx]
        ty = cy[idx]
        alpha = m.atan2((ty - self.rear_y), (tx - self.rear_x)) - state.yaw
        delta = m.atan2(
            2 * self.WB * m.sin(alpha),
            np.hypot(self.rear_x - tx, self.rear_y - ty)
        )
        delta = np.clip(delta, -self.max_steer, self.max_steer)

        return delta

class PurePursuitNode:
    def __init__(self):
        rospy.init_node("pure_pursuit_node")

        self.cx = []
        self.cy = []
        self.pp = pure_pursuit()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0 
        
        self.target_velocity = 0.0

        # Subscriber 
        rospy.Subscriber("odometry/filtered_global", Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber("/global_path", Path, self.path_callback, queue_size=10)
        rospy.Subscriber("/target_velocity", Float32, self.target_velocity_callback, queue_size=10)
        
        self.cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=10)

        # Timer (10Hz)
        rospy.Timer(rospy.Duration(0.1), self.pp_control)

    def odom_callback(self, msg):
        self.state.x = msg.pose.pose.position.x
        self.state.y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        self.state.yaw = np.arctan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        )
        # 현재 차량의 속도 (m/s)
        self.state.v = msg.twist.twist.linear.x

    def path_callback(self, msg):
        self.cx = [pose.pose.position.x for pose in msg.poses]
        self.cy = [pose.pose.position.y for pose in msg.poses]

    def target_velocity_callback(self, msg):
        self.target_velocity = msg.data / 3.6  # km/h -> m/s

    def pp_control(self, event):  
        if len(self.cx) < 2:
            return

        # 횡방향 제어 (조향각 계산)
        self.pp.calc_target_index(self.state, self.cx, self.cy)
        delta = self.pp.pure_pursuit_control(self.state, self.cx, self.cy)

        # MORAI 제어 메시지 인스턴스 생성
        cm = CtrlCmd()
        
        # 종방향 제어 타입을 1번(accel, brake)으로 설정 
        cm.longlCmdType = 1 

        self.target_velocity = 10.0 / 3.6
        
        # 종방향 제어
        if self.state.v < self.target_velocity:
            cm.accel = 0.25  # 가속 페달 개도량 (0.0 ~ 1.0)
            cm.brake = 0.0
        else:
            cm.accel = 0.0
            cm.brake = 0.30  # 브레이크 페달 개도량 (0.0 ~ 1.0)

        # MORAI 조향 단위 적용 (Radian)
        # MORAI 조향각은 라디안 단위를 그대로 받음, ROS 표준에 따라 좌회전이 (+), 우회전이 (-)
        # 기존 erp42와 달리 도(degree) 변환이나 부호 반전이 필요 없음
        cm.front_steer = delta 

        self.cmd_pub.publish(cm)

def main():
    node = PurePursuitNode()
    try:
        rospy.spin()  
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()