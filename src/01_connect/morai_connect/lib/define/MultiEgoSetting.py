from typing import List
from lib.define.type import *
from lib.define.base import Base

class Data(Base):
    _fields_ = [
        ("ego_index", _short),
        ("position_x", _float),
        ("position_y", _float),
        ("position_z", _float),
        ("roll", _float),
        ("pitch", _float),
        ("yaw", _float),
        ("velocity", _float),
        ("gear", _byte),
        ("ctrl_mode", _byte),
    ]    
    def __init__(self):
        self.ego_index = 0
        self.position_x = 0
        self.position_y = 0
        self.position_z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.velocity = 0
        self.gear = 0
        self.ctrl_mode = 0


class MultiEgoSetting(Base):
    _fields_ = [
        ("header", _char * 17),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("Num_of_Ego", _int),
        ("Cam_index", _int),
        ("_data", Data * 20), 
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = '#MultiEgoSetting$'.encode()
        self.data_lenght = 648
        self.aux_data = (0,0,0)
        self.Num_of_Ego = 0
        self.Cam_index = 0
        self._data = (Data * 20)()
        self.tail = '\r\n'.encode()  

    @property
    def data(self) -> List[Data]:
        return list(self._data)