from typing import List
from lib.define.type import *
from lib.define.base import Base

class Data(Base):
    _fields_ = [
        ("objType", _short),
        ("obj_id", _short),
        ("pose_x", _float),
        ("pose_y", _float),
        ("pose_z", _float),        
        ("globalOffset_x", _float),
        ("globalOffset_y", _float),
        ("globalOffset_z", _float),
    ]
    def __init__(self):
        self.objType = 0
        self.obj_id = 0
        self.pose_x = 0
        self.pose_y = 0
        self.pose_z = 0
        self.globalOffset_x = 0
        self.globalOffset_y = 0
        self.globalOffset_z = 0

class CollisionData(Base):
    _fields_ = [
        ("header", _char * 15),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("sec", _int),
        ("nsec", _int),
        ("_data", Data * 5),
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = b""
        self.data_lenght = 0
        self.aux_data = (0,0,0)
        self.sec = 0
        self.nsec = 0        
        self._data = (Data * 5)()

    @property
    def data(self) -> List[Data]:
        return list(self._data)        
        