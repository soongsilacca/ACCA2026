# ros2
import rclpy
from rclpy.qos import  qos_profile_system_default

# msg
from geometry_msgs.msg import PoseArray,PoseStamped,Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from erp42_msgs.msg import ControlMessage

# tf
from tf_transformations import *

# stanley
from stanley import Stanley

# DB
try:
    from DB import DB
except Exception as e:
    print(e)

# utils
import time
from enum import Enum
import numpy as np
import math as m

class SpeedSupporter:
    def __init__(self, node):
        # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value

        self.he_gain = node.declare_parameter("/speed_supporter/he_gain_parking", 50.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain_parking", 30.0).value

        self.he_thr = node.declare_parameter("/speed_supporter/he_thr_parking",0.001).value
        self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr_parking",0.002).value

    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self, value, hdr, ctr, min_value, max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)
        return res

class PID():
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain_parking", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain_parking", 0.85).value
       
        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0
        
        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)

    def PIDControl(self, speed, desired_value, min = 0, max = 25):

        self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        dt = self.current - self.last
        self.last = self.current

        err = desired_value - speed
        # self.d_err = (err - self.p_err) / dt 
        self.p_err = err
        self.i_err += self.p_err * dt  * (0.0 if speed == 0 else 1.0)

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(self.speed, min, max))
    

class Parking_state(Enum):
    SEARCH = 0
    PARKING = 1
    STOP = 2
    RETURN = 3
    FINISH = 4

class Pose:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
                
class Detection:
    def __init__(self,node,topic):
        self.node = node
        self.cone = []
        self.node.create_subscription(
            PoseArray,
            topic,
            callback = self.update,
            qos_profile = qos_profile_system_default
        )
    def update(self,msg):
        # print("detection")
        self.cone = msg.poses
    



class Parking():
    def __init__(self,node):
        self.node = node
        
        #### search_path ####
        # kcity-bs
        self.search_path_db = DB("1012_1344_bs_search_path.db") 
        # school-bunsudae
        # self.search_path_db = DB("search_path_bunsudae.db") 
        #### search_path ####

        self.search_path = self.search_path_db.read_db_n("Path", "x", "y", "yaw")
        rows = self.search_path
        self.path_cx = [row[0] for row in rows]
        self.path_cy = [row[1] for row in rows]
        self.path_cyaw = [row[2] for row in rows]
        self.search_path = np.array(self.search_path)
        
        #### parking_path ####
        # bunsudae
        # self.parking_path_db = DB("parking_path_bunsudae.db")
        # kcity
        self.parking_path_db = DB("1012_1438_parking_path.db")
        #### Parking_path ####

        self.parking_path = self.parking_path_db.read_db_n("Path", "x", "y", "yaw")
        self.parking_path = np.array(self.parking_path)

        #### return_path ####
        # self.return_path_db = DB("1009_2243_return_path.db")
        # self.return_path = self.return_path_db.read_db_n(
        #     "Path","x","y","yaw"
        # )
        # self.return_path = np.array(self.return_path)
        #### return_path ####

        # parameter
        # self.cone_dist_threshold = self.node.declare_parameter("/parking/cone_dist_threshold", 3.1)
        self.cone_dist_threshold = 3.1
        # self.euclidean_dist_threshold = self.node.declare_parameter("/parking/euclidean_dist_threshold", 1.0)
        self.euclidean_dist_threshold = 1.0
        # visualization = self.node.declare_parameter("/parking/visualization", True)
        visualization = True
        self.reference_pose = Pose(
            self.search_path[1][0], self.search_path[1][1], self.search_path[1][2]
        )  
        # ROI(rectangular form)
        self.min_x = self.reference_pose.x + 0.0 + 13.89150679316159 - 6.0- 4.5 - 5.0 + 14.0 - 5.0
        self.max_x = self.reference_pose.x + 28.0 + 13.89150679316159 -6.0 - 5.0-4.5 -5.0 -2.5 + 14.0 - 5.0
        self.min_y = self.reference_pose.y -2.5 - 0.49272527596502247 -0.8 + 0.65
        self.max_y = self.reference_pose.y  -1.0 - 0.49272527596502247 -0.5 +0.65
        
        # class variable
        self.parking_area_detected = False
        self.goal = 0 # parking_area_detected --> change
        self.vehicle_state = None
        self.parking_state = Parking_state.SEARCH
        self.target_idx = 0
        self.stop_start_time = 0
        self.roi_cone = []
        self.parking_stop_time = 0
        self.idx = None
        
        # instance
        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)
        self.dt = Detection(node,"/cone_pose_map")
        
        #visualization
        if visualization:
            self.marker_id = 1
            self.pub_path = node.create_publisher(Path,"parking_path",qos_profile_system_default)
            self.pub_roi_cone_marker = node.create_publisher(MarkerArray, "roi_cone", qos_profile_system_default)
            self.goal_pose_marker = node.create_publisher(Marker,"marker_goal",qos_profile_system_default)
            self.reference_pose_marker = node.create_publisher(Marker,"reference_pose",qos_profile_system_default)
            self.roi_marker_array = node.create_publisher(MarkerArray,"roi",qos_profile_system_default)
            self.roi_visualization()
            self.current_path_visualization()
    
    ###################### utils ##########################

    def calcluate_brake(
        self, adapted_speed
    ):  # brake 값 정하는 알고리즘 좀 더 정교하게 생각
        if self.vehicle_state.v * 3.6 >= adapted_speed:
            brake = (abs(self.state.v * 3.6 - adapted_speed) / 20.0) * 200
        else:
            brake = 0
        return np.clip(brake, 0, 200)
    
    def rotate_points(self, points, angle, origin):
        if points is not None:

            angle_radians = -angle  
            # 시계방향으로 회전
            rotation_matrix = np.array(
                [
                    [np.cos(angle_radians), -np.sin(angle_radians)],
                    [np.sin(angle_radians), np.cos(angle_radians)],
                ]
            )
            translated_points = points - origin
            rotated_points = np.dot(translated_points, rotation_matrix.T)
            rotated_points += origin
            return rotated_points

    def euclidean_duplicate(self, new_point):
        for original_point in self.roi_cone:
            distance = m.sqrt((new_point[0] - original_point[0]) ** 2 + (new_point[1] - original_point[1]) ** 2)
            if distance <= self.euclidean_dist_threshold:
                return True
        return False
    
    def in_roi(self, point):
        if self.min_x <= point[0] <= self.max_x:
            if self.min_y <= point[1] <= self.max_y:
                return True
        return False
    
    def detection(self):
        for point in [(pose.position.x, pose.position.y) for pose in self.dt.cone]:
            rotated_point = self.rotate_points(
                points = np.array([point]),
                angle = self.reference_pose.yaw,
                origin = np.array([self.reference_pose.x, self.reference_pose.y]),
            )
            # 고정된 map frame에 align
            rotated_point = tuple(rotated_point[0])
            if not self.euclidean_duplicate(rotated_point):
                if self.in_roi(rotated_point):
                    self.roi_cone.append(rotated_point)
                    self.roi_cone_visualization()
                    self.update_parking_path()

    def update_parking_path(self):
        if self.parking_area_detected is False:
            for i in range(len(self.roi_cone) - 1):
                self.roi_cone = sorted(self.roi_cone,key= lambda x : x[0]) # map frame에 Align된 cone들을 x 축 방향으로 정렬
                dist = self.roi_cone[i + 1][0] - self.roi_cone[i][0]
                print(f"{dist} m")
                if dist > self.cone_dist_threshold:
                    self.idx = i
                    self.parking_path_align()
                    self.parking_area_detected = True
                    break

    def parking_path_align(self):

        angle = self.parking_path[-1][2] - self.reference_pose.yaw
        
        goal_pose = self.rotate_points(
            np.array(
                [self.roi_cone[self.idx][0] + 2.0, self.roi_cone[self.idx][1] - 1.2]
            ),
            -self.reference_pose.yaw,
            np.array([self.reference_pose.x, self.reference_pose.y]),
        )

        dx = goal_pose[0] - self.parking_path[0][0]
        dy = goal_pose[1] - self.parking_path[0][1]

        self.parking_path[:, 0] += dx
        self.parking_path[:, 1] += dy
        
        self.parking_path[:, :2] = self.rotate_points(
            self.parking_path[:, :2], angle, self.parking_path[0][:2]
        )  
        
        parking_start_index_in_search_path = self.search_path_db.find_idx(
            self.parking_path[-1][0], self.parking_path[-1][1], "Path"
        )
        self.goal = len(self.search_path) - parking_start_index_in_search_path - 1
        self.node.get_logger().info(f"{self.goal} is made")
        self.goal_pose_visualization(idx = parking_start_index_in_search_path) 
       
    ###################### utils ##########################
    



    def control_parking(self,State):
        self.vehicle_state = State

        self.node.get_logger().info(f"{self.parking_state},{self.target_idx},{len(self.path_cx) - 1}")
        
        if self.parking_state == Parking_state.SEARCH:
            self.detection()
            
        if self.target_idx >= len(self.path_cx) - self.goal - 1:
            
            # 이 아레 조건문은 대략 이런뜻을 가진다, " 현재 다음과 같은 parking_state인데 다음 state로 전환할거다"

            if self.parking_state == Parking_state.SEARCH:

                self.parking_state = Parking_state(self.parking_state.value + 1)  # SEARCH -> PARKING
                self.goal = 0
                self.path_cx = self.parking_path[::-1, 0]  # 첫 번째 열 (cx 값들)
                self.path_cy = self.parking_path[::-1, 1]  # 두 번째 열 (cy 값들)
                self.path_cyaw = self.parking_path[::-1, 2]  # 세 번째 열 (cyaw 값들)
                self.current_path_visualization()
                self.target_idx = 0
                self.parking_stop_time = time.time()
                
                msg = ControlMessage(mora=0, estop=1,gear=0,speed = 0*10, steer = 0,brake=200)
                return msg, False

            elif self.parking_state == Parking_state.PARKING:
                self.parking_state = Parking_state(self.parking_state.value + 1)  # PARKING -> STOP #TODO:return_path는 안에서 밖으로 가는걸로 다시만들기(조향이 달라서 그런지 잘 못 따라감
                self.stop_start_time = time.time()  # STOP 상태로 전환된 시간을 기록
                self.target_idx = 0
                msg = ControlMessage(mora=0, estop=1,gear=2,speed = 0*10, steer = 0,brake=200)
                return msg, False
            
            elif self.parking_state == Parking_state.RETURN:
                msg = ControlMessage(mora=0, estop=1,gear=2,speed = 0*10, steer = 0,brake=200) #TODO 이부분 제거 가능?
                return msg,True
            

        
        else:

            # 이 아레 조건문은 대략 이런뜻을 가진다, " 현재 다음과 같은 parking_state이다, parking_state에 맞는 행동을 하겠다"

            if self.parking_state == Parking_state.STOP:

                if time.time() - self.stop_start_time >= 3.0:
                    self.node.get_logger().info("STOP state finished, moving to RETURN state")
                    self.parking_state = Parking_state(self.parking_state.value + 1)
                    self.target_idx = 0  # STOP -> RETURN
                self.node.get_logger().info(f"estop time_stop {time.time() - self.stop_start_time}")
                self.search_path = np.array(self.search_path)
                self.path_cx = self.parking_path[::1, 0]  # 첫 번째 열 (cx 값들)
                self.path_cy = self.parking_path[::1, 1]  # 두 번째 열 (cy 값들)
                self.path_cyaw = self.parking_path[::1, 2]  # 세 번째 열 (cyaw 값들)
                # self.path_cx = self.search_path[::1, 0]
                # self.path_cy = self.search_path[::1, 1]
                # self.path_cyaw = self.search_path[::1, 2]
                self.current_path_visualization()
                msg = ControlMessage(mora=0, estop=1,gear=0,speed = 0*10, steer = 0,brake=200)

                return msg, False

            # SEARCH, PARKING, RETURN
            elif self.parking_state == Parking_state.SEARCH:    
                try:
                    steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                        State,
                        self.path_cx,
                        self.path_cy,
                        self.path_cyaw,
                        h_gain=0.5,
                        c_gain =0.24,
                        reverse=False,
                    )


                    target_speed = 8.0 # 아레와 맞지 않음 original 5.0
                    adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=4, max_value=6)
                    speed = self.pid.PIDControl(State.v * 3.6, adapted_speed, min=4, max=6) # original 4 , 10
                    msg = ControlMessage(mora=0, estop=0,gear=2,speed = speed*10, steer = int(m.degrees(-1* steer) * 1e3),brake=0)

                except Exception as e:
                    print(f"{e}: stanley")
                return msg, False
            
            # Kcity 경사로 setting                
            elif self.parking_state == Parking_state.PARKING:
                if time.time() - self.parking_stop_time <= 0.05: # DELERE IF NOT NECESSARY
                    msg = ControlMessage(mora=0, estop=1,gear=0,speed = 0*10, steer = 0,brake=200)
                    return msg, False                 
                    
                try:
                    steer, self.target_idx, hdr,ctr = self.st.stanley_control(
                        State,
                        self.path_cx,
                        self.path_cy,
                        self.path_cyaw,
                        h_gain = 1.5,
                        c_gain = 1.2,
                        reverse=True,
                    )
                    target_speed = 2.0
                    adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=2, max_value=3
                    )
                    brake = self.calcluate_brake(adapted_speed)
                    msg = ControlMessage(mora=0, estop=0,gear=0,speed = int(adapted_speed)*10, steer = int(m.degrees(-1* steer) * 1e3),brake= int(brake))
                except Exception as e:
                    print(f"Stanley:{e}\n",f"{State}")
                return msg, False
            
            # elif self.parking_state == Parking_state.PARKING:                
            #     if time.time() - self.parking_stop_time <= 0.2:
            #         msg = ControlMessage(mora=0, estop=1,gear=0,speed = 0*10, steer = 0,brake=200)
            #         self.node.get_logger().info(f"estop time_stop {time.time() - self.stop_start_time}")                 
            #         return msg, False                    
            #     try:
            #         steer, self.target_idx, hdr,ctr = self.st.stanley_control(
            #             State,
            #             self.path_cx,
            #             self.path_cy,
            #             self.path_cyaw,
            #             h_gain = 2.0,
            #             c_gain = 1.5,
            #             reverse=True,
            #         )
            #         # target_speed = 3.0
            #         target_speed = 10.0
            #         self.goal = 5

            #         adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=8, max_value=10
            #         )
            #         speed = self.pid.PIDControl(State.v * 3.6, adapted_speed,min=8, max=10)
            #         brake = self.calcluate_brake(adapted_speed)
            #         msg = ControlMessage(mora=0, estop=0,gear=0,speed = speed*10, steer = int(m.degrees(-1* steer) * 1e3),brake= int(brake))
            #     except Exception as e:
            #         print(f"Stanley:{e}\n",f"{State}")
            #     return msg, False
            
            else: # RETURN
                try:
                    steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                        State,
                        self.path_cx,
                        self.path_cy,
                        self.path_cyaw,
                        h_gain=2.5,
                        c_gain =3.2, # original 2.2
                        reverse=False,
                    )


                    target_speed = 7.0 # ORIGINAL 7.0 MIN 6  MAX 8
                    adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=6, max_value=8
                    )
                    speed = self.pid.PIDControl(State.v * 3.6, adapted_speed, min = 6 , max = 8)
                    msg = ControlMessage(mora=0, estop=0,gear=2,speed = int(adapted_speed)*10, steer = int(m.degrees(-1* steer) * 1e3),brake=0)

                except Exception as e:
                    print(f"{e}: stanley")
                return msg, False

    ###################### visualization ##########################

    def goal_pose_visualization(self, idx):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        
        marker.action = Marker.ADD
        # Marker의 위치 설정
        marker.pose.position.x = self.search_path[idx][0]
        marker.pose.position.y = self.search_path[idx][1]
        marker.pose.position.z = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
         # Marker의 크기 설정 (x, y, z 방향의 크기)
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # Marker의 색상 설정 (R, G, B, A 값)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Marker의 수명을 무한대로 설정
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        # 퍼블리시
        self.goal_pose_marker.publish(marker)
        
    def roi_visualization(self):
        combinations = [
        (self.min_x, self.min_y),  # Bottom-left
        (self.max_x, self.min_y),  # Bottom-right
        (self.max_x, self.max_y),  # Top-right
        (self.min_x, self.max_y),  # Top-left
        ]
        combinations = np.array(combinations)

        combinations = self.rotate_points(combinations,-self.reference_pose.yaw,np.array([self.reference_pose.x,self.reference_pose.y]))

        
        # Create marker array for both spheres and the line strip
        marker_array = MarkerArray()

        # Create a line strip marker for the outline
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = self.node.get_clock().now().to_msg()
        line_marker.ns = "roi"
        line_marker.id = self.marker_id
        self.marker_id += 1  # Increment unique marker ID
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0  # Identity quaternion
        line_marker.scale.x = 0.25  # Thickness of the lines
        line_marker.color.a = 1.0
        line_marker.color.r = 1.0  # Set the line color to red
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0

        # Add the corner points to the line strip
        for x, y in combinations:
            point = Point()
            point.x = x
            point.y = y
            point.z = -1.0
            line_marker.points.append(point)

        # Close the loop by adding the first point again
        first_point = Point()
        first_point.x = combinations[0][0]
        first_point.y = combinations[0][1]
        first_point.z = 0.0
        line_marker.points.append(first_point)

        # Add the line strip to the marker array
        marker_array.markers.append(line_marker)



        # Publish the marker array
        self.roi_marker_array.publish(marker_array)
    
    def roi_cone_visualization(self):
        if self.roi_cone:
            marker_array_ = MarkerArray()
            origin_roi_cone = self.rotate_points(
                np.array(self.roi_cone),
                -self.reference_pose.yaw,
                np.array([self.reference_pose.x, self.reference_pose.y]),
            )
            for x, y in origin_roi_cone:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.node.get_clock().now().to_msg()
                marker.ns = "cone"
                marker.id = self.marker_id
                self.marker_id += 1  # 고유한 마커 ID 설정
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x  # 각 마커를 서로 다른 위치에 배치
                marker.pose.position.y = y
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

                marker_array_.markers.append(marker)
            self.pub_roi_cone_marker.publish(marker_array_)
    
    def current_path_visualization(self):
        path = Path()
        path.header = Header()
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for x, y, yaw in zip(self.path_cx,self.path_cy,self.path_cyaw):
                pose = PoseStamped()
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                quaternion = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                path.poses.append(pose)
        self.pub_path.publish(path)

    ###################### visulization ##########################
