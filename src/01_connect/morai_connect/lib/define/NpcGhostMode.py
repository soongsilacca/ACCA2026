from typing import List
from lib.define.type import *
from lib.define.base import Base

class Data(Base):
    _fields_ = [        
        ("unique_id", _int8),  
        ("car_name", _char * 25),
        ("pose_x", _float),
        ("pose_y", _float),
        ("pose_z", _float),        
        ("roll", _float),
        ("pitch", _float),
        ("yaw", _float)
    ]

    def __init__(self):
        self.unique_id = 0
        self.car_name = b''
        self.pose_x = 0
        self.pose_y = 0
        self.pose_z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

class NpcGhostMode(Base):

    _fields_ = [
        ("header", _char * 13),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("_data", Data * 20),
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = '#NpcGhostCmd$'.encode()
        self.data_lenght = 1000
        self.aux_data = (0,0,0)
        self._data = (Data * 20)()
        self.tail = '\r\n'.encode()  

    @property
    def data(self) -> List[Data]:
        return list(self._data)