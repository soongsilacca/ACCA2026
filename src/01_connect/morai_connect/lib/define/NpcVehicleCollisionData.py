from typing import List
from lib.define.type import *
from lib.define.base import Base

class Data(Base):
    _fields_ = [
        ("objType_1", _short),
        ("obj_id_1", _short),        
        ("pose_x_1", _float),
        ("pose_y_1", _float),
        ("pose_z_1", _float),
        ("heading_1", _float),
        ("size_x_1", _float),
        ("size_y_1", _float),
        ("size_z_1", _float),        
        ("vel_x_1", _float),
        ("vel_y_1", _float),
        ("vel_z_1", _float),
        ("accel_x_1", _float),
        ("accel_y_1", _float),
        ("accel_z_1", _float),        
        ("objType_2", _short),
        ("obj_id_2", _short),        
        ("pose_x_2", _float),
        ("pose_y_2", _float),
        ("pose_z_2", _float),
        ("heading_2", _float),
        ("size_x_2", _float),
        ("size_y_2", _float),
        ("size_z_2", _float),        
        ("vel_x_2", _float),
        ("vel_y_2", _float),
        ("vel_z_2", _float),
        ("accel_x_2", _float),
        ("accel_y_2", _float),
        ("accel_z_2", _float),
    ]

    def __init__(self):
        self.objType_1 = 0
        self.obj_id_1 = 0
        self.pose_x_1 = 0
        self.pose_y_1 = 0
        self.pose_z_1 = 0
        self.heading_1 = 0
        self.size_x_1 = 0
        self.size_y_1 = 0
        self.size_z_1 = 0
        self.vel_x_1 = 0
        self.vel_y_1 = 0
        self.vel_z_1 = 0
        self.accel_x_1 = 0
        self.accel_y_1 = 0
        self.accel_z_1 = 0
        self.objType_2 = 0
        self.obj_id_2 = 0
        self.pose_x_2 = 0
        self.pose_y_2 = 0
        self.pose_z_2 = 0
        self.heading_2 = 0
        self.size_x_2 = 0
        self.size_y_2 = 0
        self.size_z_2 = 0
        self.vel_x_2 = 0
        self.vel_y_2 = 0
        self.vel_z_2 = 0
        self.accel_x_2 = 0
        self.accel_y_2 = 0
        self.accel_z_2 = 0

class NpcVehicleCollisionData(Base):
    _fields_ = [
        ("header", _char * 18),
        ("data_lenght", _int),
        ("aux_data", _int * 3),        
        ("_data", Data * 10), 
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = '#VehicleCollision$'.encode()
        self.data_lenght = 0
        self.aux_data = (0,0,0)        
        self._data = (Data * 10)()
        self.tail = '\r\n'.encode()  

    @property
    def data(self) -> List[Data]:
        return list(self._data)