from typing import List
from lib.define.type import *
from lib.define.base import Base

class Data(Base):
    _fields_ = [
        ("obj_id", _short),
        ("objType", _short),
        ("pose_x", _float),
        ("pose_y", _float),
        ("pose_z", _float),
        ("heading", _float),
        ("size_x", _float),
        ("size_y", _float),
        ("size_z", _float),
        ("overhang", _float),
        ("wheelbase", _float),
        ("rearoverhang", _float),
        ("vel_x", _float),
        ("vel_y", _float),
        ("vel_z", _float),
        ("accel_x", _float),
        ("accel_y", _float),
        ("accel_z", _float),
        ("link_id", _char * 38)
    ]

    def __init__(self):
        self.obj_id = 0
        self.objType = 0
        self.pose_x = 0
        self.pose_y = 0
        self.pose_z = 0
        self.heading = 0
        self.size_x = 0
        self.size_y = 0
        self.size_z = 0
        self.overhang = 0
        self.wheelbase = 0
        self.rearoverhang = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.link_id = b''


class ObjectInfo(Base):
    _fields_ = [
        ("header", _char * 14),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("sec", _int),
        ("nsec", _int),
        ("_data", Data * 20),
        ("tail", _char * 2)

    ]

    def __init__(self):
        self.header = b''
        self.data_lenght = 0
        self.aux_data = (0,0,0)
        self.sec = 0
        self.nsec = 0  
        self._data = (Data * 20)()

    @property
    def data(self) -> List[Data]:
        return list(self._data)