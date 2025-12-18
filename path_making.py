import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import os
import sqlite3
from erp42_msgs.msg import SerialFeedBack, ControlMessage
import numpy as np
np.float = float

import math as m
from scipy.interpolate import CubicSpline

from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker

try:
    from DB import DB as db
except Exception as e:
    print(e)

class DBWRITE(Node):
    def __init__(self):
        super().__init__("dbwrite")

        self.estop = 0
        self.estop_positions = []
        self.count = 0
        self.idx = 0


        self.sub_local = self.create_subscription(
            Odometry, "localization/kinematic_state", self.callback_local, qos_profile_system_default
        )

        self.sub_feedback = self.create_subscription(
            SerialFeedBack,"erp42_feedback",self.callback_feedback,qos_profile_system_default
        )
        
        self.pub_marker_array = self.create_publisher(
            MarkerArray,"domain",qos_profile_system_default
        )
        self.pub_estop_marker_array = self.create_publisher(
            MarkerArray,"estop_position",qos_profile_system_default
        )        
        
        #############################################################
        db_dir = "/home/acca/db_file"
        db_name = "0827_ssupark_ys.db"
        #############################################################

        self.marker_timer = self.create_timer(1.0,self.domain_for_visu)
        self.path_x = []
        self.path_y = []
        self.path_cov = []
        self.path = []
        self.euclidean_list = []
        self.distance = 0
        self.ds = 0.1
        


        db_path = os.path.join(db_dir, db_name)

        # DB 파일 존재 여부 확인 후 처리
        # 1) DB 파일 존재 여부 확인 & 테이블 생성
        while True:
            if os.path.isfile(db_path):
                answer = input(f"Database Found At {db_path}. Delete and recreate? (Y/N) or enter new name: ")
                if answer.lower() == 'y':
                    os.remove(db_path)
                    print(f"Deleted existing database at {db_path}.")
                elif len(answer) > 2:
                    # 파일 이름 직접 입력
                    if answer[-3:] != ".db":
                            answer += ".db"

                    db_path = os.path.join(db_dir, answer)
                    print(f"Creating new database at {db_path}.")
                else:
                    print("Exiting without creating database.")
                    return

            # 파일이 없거나 삭제된 경우, 새로 생성
            print(f"Writing new DB at {db_path}")
            self.__conn = sqlite3.connect(db_path)
            self.__cur  = self.__conn.cursor()
            # 테이블 생성 SQL
            self.__cur.execute("""
                CREATE TABLE IF NOT EXISTS Node(
                    Start_point CHAR(4),
                    End_point   CHAR(4),
                    path_id     CHAR(4) PRIMARY KEY,
                    mission     CHAR(10)
                );
            """)
            self.__cur.execute("""
                CREATE TABLE IF NOT EXISTS Path(
                    path_id CHAR(4),
                    idx     INTEGER PRIMARY KEY,
                    x       REAL,
                    y       REAL,
                    yaw     REAL,
                    speed   REAL,
                    FOREIGN KEY(path_id) REFERENCES Node(path_id)
                        ON DELETE CASCADE ON UPDATE CASCADE
                );
            """)
            self.__conn.commit()
            break
        self.db = db(db_name) # the name of DB
        for i in range(1,31,1):
            self.db.write_db_Node([(f"A{i}",f"A{i+1}",f"A{i}A{i+1}"),])
        
    def callback_local(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 경로 점 추가
        if not self.euclidean_duplicate((x,y)):
            self.path_x.append(x)
            self.path_y.append(y)

        # estop 이벤트 발생 시 기록
        if self.estop == 1 and self.count == 0:
            self.estop_positions.append((x, y))
            self.count = 1
        elif self.estop == 0:
            self.count = 0
            print(self.estop_positions)

        
            
        # else:
            
        #     euc_dup_list = sorted(self.euclidean_list, key=lambda point: point[2])
        #     low_cov_point = euc_dup_list[0]
            
        #     if low_cov_point == p1:
        #         self.update_path_with_low_cov_point(euc_
        # dup_list)
        #     self.euclidean_list.clear()
            
        if len(self.path_x) >= 3:
            self.interpolate_path()

    def callback_feedback(self, msg: SerialFeedBack):
        # estop 신호 수신
        self.estop = msg.estop

    def euclidean_duplicate(self, p1): # 현재 위치와 이전 기록 간의 유클리드 거리가 threshhold보다 작은 점인지 판단하기
        threshold = 1.0
        # a=0
        for x, y in zip(self.path_x, self.path_y):
            distance = m.sqrt((p1[0] - x) ** 2 + (p1[1] - y) ** 2)
            if distance <= threshold:
            #     a=1
            #     self.euclidean_list.append((x, y, cov))
            # if a==1:
            #     self.euclidean_list.append(p1)    
                return True
        return False

    # def update_path_with_low_cov_point(self, euc_dup_list):
    #     low_cov_point = euc_dup_list[0]
    #     for x, y, cov in euc_dup_list[1:]:
    #         if x in self.path_x:
    #             self.path_x.insert(self.path_x.index(x),low_cov_point[0])
    #             self.path_x.remove(x)
    #         if y in self.path_y:
    #             self.path_y.insert(self.path_y.index(y),low_cov_point[1])
    #             self.path_y.remove(y)
    #         if cov in self.path_cov:
    #             self.path_cov.insert(self.path_cov.index(cov),low_cov_point[2])
    #             self.path_cov.remove(cov)

    def interpolate_path(self):
        x = np.array(self.path_x)
        y = np.array(self.path_y)
        dx_ = np.diff(x) # path_x의 차이 array
        dy_ = np.diff(y) # path_y의 차이 array
        ds = np.sqrt(dx_**2 + dy_**2) # path point간의 거리
        s = np.concatenate([[0], np.cumsum(ds)])
        try:
            cs_x = CubicSpline(s, x, bc_type="natural")
            cs_y = CubicSpline(s, y, bc_type="natural")
            
            self.narrow = int(s[-1] / self.ds)

            s_new = np.linspace(s[0], s[-1], self.narrow)
            
            self.get_logger().info(f"path_idx_num: {self.narrow}.")
            x_new = cs_x(s_new)
            y_new = cs_y(s_new)

            dx_new = cs_x(s_new, 1)
            dy_new = cs_y(s_new, 1)
            yaw_new = [m.atan2(dy, dx) for dy, dx in zip(dy_new, dx_new)]

            self.path = list(zip( x_new.tolist(), y_new.tolist(), yaw_new))
            self.write_db()
        except Exception as e:
            self.get_logger().error(f"An error occurred during spline interpolation: {e}")
            
    def write_db(self):
        self.db.write_db_Path(self.path)
        
    def create_path_markers(self):
        """빨간색 global_path 마커 생성"""
        marker_array = MarkerArray()
        for i, (px, py) in enumerate(zip(self.path_x, self.path_y)):
            mkr = Marker()
            mkr.header.frame_id = 'map'
            mkr.header.stamp = self.get_clock().now().to_msg()
            mkr.ns = 'global_path'
            mkr.id = i
            mkr.type = Marker.SPHERE
            mkr.action = Marker.ADD
            mkr.pose.position.x = px
            mkr.pose.position.y = py
            mkr.pose.position.z = 0.0
            mkr.scale.x = 1.0
            mkr.scale.y = 1.0
            mkr.scale.z = 1.0
            mkr.color.r = 1.0
            mkr.color.g = 0.0
            mkr.color.b = 0.0
            mkr.color.a = 1.0
            marker_array.markers.append(mkr)
        return marker_array

    def create_estop_markers(self):
        """파란색 estop 이벤트 마커 생성"""
        marker_array = MarkerArray()
        base_id = 0
        for j, (ex, ey) in enumerate(self.estop_positions):
            mkr = Marker()
            mkr.header.frame_id = 'map'
            mkr.header.stamp = self.get_clock().now().to_msg()
            mkr.ns = 'estop'
            mkr.id = base_id + j
            mkr.type = Marker.SPHERE
            mkr.action = Marker.ADD
            mkr.pose.position.x = ex
            mkr.pose.position.y = ey
            mkr.pose.position.z = 0.0
            mkr.scale.x = 2.0
            mkr.scale.y = 2.0
            mkr.scale.z = 2.0
            mkr.color.r = 0.0
            mkr.color.g = 0.0
            mkr.color.b = 1.0
            mkr.color.a = 1.0
            marker_array.markers.append(mkr)
        return marker_array

    def domain_for_visu(self):
        """global path와 estop markers를 합쳐 publish"""
        path_markers = self.create_path_markers()
        estop_markers = self.create_estop_markers()
        self.pub_estop_marker_array.publish(estop_markers)
        self.pub_marker_array.publish(path_markers)

    def estop_separator(self):
        # 예선
        # node = [(1, 2.226859883518016, 3.0159049392046944), (2, 41.69461113854053, 78.97310235558686), (3, 61.7708037526975, 85.6664146461356), (4, 100.70225850625832, 65.33254374611164), (5, 116.23281176167212, 57.129936831185645), (6, 203.21884124893512, 122.49950356374443), (7, 203.33345306715614, 154.86169284253754), (8, 212.6167990251267, 169.23868718039154), (9, 215.41528080390003, 54.713514244168344), (10, 179.33433332537857, -30.302592801142083), (11, 129.95709495490001, -61.146144837148995), (12, 55.14701879878294, -101.10897425625137), (13, -2.145724656823697, -4.749248591806014)]
        
        #본선
        # node = [(1, 2.084378260967995, 2.9267700984394356), (2, 25.62834681494384, 49.2170596100512), (3, 38.11960926068759, 65.92836572050876), (4, 41.874925074068585, 79.52903383279012), (5, 38.69216161731938, 102.3937937188615), (6, 21.549497069511375, 110.12172017158227), (0,13.887076377868652, 135.37652587890625) ,(7, 19.953213935582028, 146.81008069972435), (8, 24.719449054313056, 154.96589615657146), (9, 37.030208406109324, 155.24994200784514), (10, 49.11942929077692, 148.51662171125156),
        #          (11, 77.44231986104529, 147.70441223230026), (12, 86.77204759982024, 165.07123225517915), (13, 132.08522534987483, 250.47736373595052), (14, 133.28787102870868, 265.27937505692313), (15, 138.52974533635975, 274.34043754797045), (16, 140.20307926358498, 290.4825371174533), (17, 140.17526716688255, 311.12597912077376), (18, 143.10142406567073, 350.9963993566367), (19, 135.7967961332568, 374.7514883383461), (20, 135.12169079455504, 395.2649702340587), 
        #          (21, 109.529648838258, 433.11383779615363), (22, 90.99546810230355, 432.1481288586937), (23, 68.37611400084963, 411.53411313520354), (24, 70.43771503842697, 343.8893886084694),[0,90.95264434814453, 307.9122009277344], (25, 114.80115098608941, 308.1832885606679), (26, 129.56530070206367, 293.57809217260797), (27, 110.13537691418031, 226.33702263187627), (28, 98.62738502904924, 203.3945724355536), (29, 87.07285588828645, 175.68631782283413), (30, 79.13417683150769, 160.62943654332494), (31, 36.674680166894724, 80.2994930332112)]
        # node = [(1, -18.335913941500934, -35.17135755335897), (2, 21.157312623662072, 40.81593555964156), (3, 41.22429702039536, 47.47330046495313), (4, 80.21738828644838, 27.09033002243386), (5, 95.67520427421806, 18.931090143854696), (6, 182.676606459646, 84.40827823591775), (7, 182.79419029866378, 116.68058187739982), (8, 192.06130222006675, 131.0258235012654), (9, 194.85724142244715, 16.526868573514754), (10, 158.7589047039884, -68.50774803022583), (11, 109.34103030344195, -99.40179767667067), (12, 34.57099414899209, -139.31309663250244), (13, -22.70124998940109, -42.925999429336265)]
        # [(1, -18.335913941500934, -35.17135755335897), (2, 21.157312623662072, 40.81593555964156), (3, 41.22429702039536, 47.47330046495313), (4, 80.21738828644838, 27.09033002243386), (5, 95.67520427421806, 18.931090143854696), (6, 182.676606459646, 84.40827823591775), (7, 182.79419029866378, 116.68058187739982), (8, 192.06130222006675, 131.0258235012654), (9, 194.85724142244715, 16.526868573514754), (10, 158.7589047039884, -68.50774803022583), (11, 109.34103030344195, -99.40179767667067), (12, 34.57099414899209, -139.31309663250244), (13, -22.70124998940109, -42.925999429336265)]
        # node = [(1, 16335.59431143119, 28475.00138389761), (2, 16338.15844157049, 28459.855081910915), (3, 16338.55795774061, 28457.471985349275)]
        # node = [(1, 2.4027011370125844, 2.91299077144292), (2, 42.70680887751672, 80.78338582679096), (3, 67.46491637003137, 82.92762420938027), (4, 203.50099698100797, 125.31602353212719), (5, 213.3414513908766, 149.04823537242498), (6, 173.76317385459612, -35.50062067866106), (7, 66.57179136165938, -98.49876282624763)]
        # node = [(1, -18.23921557407822, -35.36064092865631), (2, 22.216028422501736, 41.98294406115804), (3, 44.30248074612337, 45.959671228521074), (4, 182.7836298755737, 76.93391801437679), (5, 159.94693449761178, -67.80196480909964), ( 0, 35.795875549316406,-139.5904998779297), (6, -17.006026606399512, -35.50766487658391)]
        # node = [(1, -201.79812629717617, -41.51588007858748), (2, -207.42221880932038, -43.081007746811764), (3, -221.00513221462683, -44.313592288696505), (4, -219.165784121088, -43.78074694817355), (5, -217.37055288937844, -47.853831908108646)]
        # node = [(1, -202.27628681416334, -41.4297649827253), (2, -210.07715381445408, -43.41090867580225), (3, -223.22067438256312, -47.14192241815093), (4, -227.49281426846358, -48.492945766905976)]
        node = self.estop_positions
        
        b=[]
        print(len(node))
        for i in range(len(node)):
            a = self.db.find_idx(node[i][0],node[i][1],"Path")
            print(a)
            b.append(a)
        # b.append(231)
        print(b)

        # last index not put ok?
        print(len(b))
        # [0, 47, 179, 391, 822, 1226, 1613]

        # c=[0,102, 197, 308, 377, 591, 774, 922, 1002, 1186, 1223, 1404, 1433, 1546, 1610, 1669, 1826, 1927, 2836, 2910, 3003, 3072, 3356,3562]
        # print(len(c))
        for i in range(0,len(b)-1):
            if i == 32:
                self.db.splitPath(b[i],b[i+1],f"A{i+2}A{i+3}")
            else:
                self.db.splitPath(b[i],b[i+1],f"A{i+1}A{i+2}")




def main(args=None):
    rclpy.init(args=args)
    node = DBWRITE()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.estop_separator()
        #node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
