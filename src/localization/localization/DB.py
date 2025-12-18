import sqlite3
import os

from std_msgs.msg import Header
from nav_msgs.msg import Path
from tf_transformations import *
from geometry_msgs.msg import PoseStamped

class DB():
    def __init__(self, db_name):
        
        self.db_path = "/home/acca/db_file"+"/"+db_name
        
        already_exist = os.path.isfile(self.db_path) #파일이 이미 존재했는지 확인
        
        self.__conn = sqlite3.connect(self.db_path) #sql을 db와 연결
        
        self.__cur = self.__conn.cursor() # 커서( sql 데이터베이스 매니저), sql 명령어들을 실해해준다
        
        self.id = "A1A2" # id를 고정(조건문을 통해 자동으로 변하게 하거나 직접 바꾸면서 db를 만드는 과정을 통해 db 제작 예정)
        
        self.i =0 # table에 데이터를 쌓을때 사용하는 인덱스
        
        if already_exist: 
            print(f"Database Found At {self.db_path}")
        
        else:
            print(f"Database Not Found At {self.db_path}")
            
            print("Writing")
            
            self.makeTable() # 만약 완전 새로운 db라면 기초적인 테이블을 만든다

    # def find_idx(self,x,y,table):# idx ,id
    #     self.__cur.execute(f"SELECT idx,x,y FROM {table}")
    #     rows = self.__cur.fetchall()
    #     # print(rows)
    #     min_err  = 10000
    #     idx = 0
    #     for row in rows:
    #         x_value = row[1]
    #         y_value = row[2]
            
    #         x_err = (x_value - x)**2
    #         y_err = (y_value - y)**2
    #         total_err = x_err + y_err
            
    #         if total_err < min_err:
    #             min_err = total_err
    #             idx = row[0]
    #     return idx



    def find_idx(self,x,y,table):# idx ,id
        radius = 10
        query = f"""SELECT idx, x, y
                FROM {table}
                WHERE x BETWEEN ? AND ?
                AND y BETWEEN ? AND ?
                """
        self.__cur.execute(query, (x - radius, x + radius, y - radius, y + radius))
        rows = self.__cur.fetchall()
        min_err = float('inf')
        closest_idx = 0
        for row in rows:
            x_value = row[1]
            y_value = row[2]
            
            x_err = (x_value - x)**2
            y_err = (y_value - y)**2
            total_err = x_err + y_err
            
            if total_err < min_err:
                min_err = total_err
                closest_idx = row[0]
        return closest_idx
        
    def read_db_n(self,table,*n): # table(Path or Node) 에 해당하는 데이터를 모두 가져온다, 반환 형태 : [(,,,),(,,,), ....] 데이터 개수는 db에 따라 다름
        n_str = ', '.join(n)
        query = f"SELECT {n_str} FROM {table}"
        self.__cur.execute(query)
        rows = self.__cur.fetchall()
        # print(rows)
        return rows
    
    def makeTable(self):
        self.__cur.execute(
            "CREATE TABLE Node(Start_point CHAR(4), End_point CHAR(4), path_id CHAR(4) PRIMARY KEY);"
        ) #Node 테이블 
        self.__cur.execute(
            "CREATE TABLE Path(path_id CHAR(4), FOREIGN KEY(path_id) REFERENCES Node(path_id) ON DELETE CASCADE ON UPDATE CASCADE ,idx INTEGER, x REAL, y REAL, yaw REAL);"
            ) # Path 테이블      
        
        
    def write_db_Path(self,data): # Path에 해당하는 데이터를 Path 테이블에 기록한다 // data = [("",int,float,float,float), (,,,,)  .... ] 과 같은 형태 리스트 인덱스 순으로 기록된다. idx값이 같다면 새로운 데이터로 덮어쓴다
        for i,x,y,yaw in enumerate(data):
            self.__cur.execute(
                    "INSERT INTO Path (path_id,idx, x, y, yaw) VALUES (?, ?, ?, ?,?) ON CONFLICT(idx) DO UPDATE SET path_id = excluded.path_id, value_x=excluded.value_x, value_y=excluded.value_y, yaw=excluded.yaw",
                    (self.id, i , x, y, yaw),
                )
        self.__conn.commit()
    
    
    def write_db_Path_con(self,data): # 덮어쓰지않고 data를 다음줄에 추가한다
        for x,y,yaw in data:
            self.__cur.execute(
                    "INSERT INTO Path (path_id,idx, x, y, yaw) VALUES (?, ?, ?, ?,?)",
                    (self.id, self.i , x, y, yaw),
                )
            self.i += 1
        self.__conn.commit()
    
    
    def write_db_Node(self,data): # Node 테이블에 데이터를 추가한다 // data = [("","",""),(,,,) .... ]
        for str,end,id,in data:
            self.__cur.execute(
                "INSERT INTO Node (Start_point, End_point,path_id) VALUES (?,?,?)",
                (str,end,id),
            ) 
        self.__conn.commit()
        
        
    # def read_db_all(self,table): # table(Path or Node) 에 해당하는 데이터를 모두 가져온다, 반환 형태 : [(,,,),(,,,), ....] 데이터 개수는 db에 따라 다름
    #     self.__cur.execute("SELECT x,y,yaw FROM %s",table)
    #     rows = self.__cur.fetchall()
    #     print(rows)
    #     return rows
    
    
    def query_from_id_to_path(self,id): #id를 통해 Path 테이블의 x,y,yaw 값을 가져온다, 반환 형태: ros2 nav_msgs/msg/Path
        self.__cur.execute("SELECT x,y,yaw FROM Path where path_id == ",(id,))
        rows = self.__cur.fetchall()
        
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for id, x, y, yaw in rows:
            # if id == "b2c1":
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
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
        return path
    
    def query_from_id(self,id):
        self.__cur.execute("SELECT x,y,yaw,speed FROM Path where path_id == ?",(id,))
        rows = self.__cur.fetchall()
        
        cx = []
        cy = []
        cyaw = []
        cv = []

        for x, y, yaw, v in rows:
            cx.append(x)
            cy.append(y)
            cyaw.append(yaw)
            cv.append(v)
        
        return cx, cy, cyaw, cv

    def query_from_id_mpc(self, id):
        self.__cur.execute("SELECT x,y,yaw,speed, idx FROM Path where path_id == ?",(id,))
        rows = self.__cur.fetchall()
        
        cx = []
        cy = []
        cyaw = []
        cv = []
        cidx = []

        for x, y, yaw, v, idx in rows:
            cx.append(x)
            cy.append(y)
            cyaw.append(yaw)
            cv.append(v)
            cidx.append(idx)
            
        return cx, cy, cyaw, cv, cidx
    
    
    def read_db_from_id_to_mission(self,id): # id 를 통해 Node테이블의 mission 정보를 가져온다
        self.__cur.execute("SELECT mission FROM Node where path_id == ",(id,))
        rows = self.__cur.fetchone()
        return rows
        
    def deletePath(self, path_id): # id에 해당하는 데이터를 지운다(), foreign key의 제약조건이 CASCADE이므로 노드테이블의 id데이터가 지워지면 같이 지워짐
        # query_Path = "DELETE FROM Path WHERE path_id = ?;"
        query_Node = "DELETE FROM Node WHERE path_id = ?;"
        # self.__cur.execute(query_Path, (path_id,))
        self.__cur.execute(query_Node, (path_id,))
        self.__conn.commit()


        
    def __del__(self):
        # Destructor to ensure the connection is closed when the object is deleted
        if self.__conn:
            self.__conn.close()