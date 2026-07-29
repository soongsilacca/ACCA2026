from lib.define.type import *
from lib.define.base import Base

class EgoCtrlCmd(Base):
    _fields_ = [
        ("header", _char * 14),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("ctrl_mode", _int8),
        ("gear", _int8),
        ("cmd_type", _int8),
        ("velocity", _float),
        ("acceleration", _float),
        ("accel", _float),
        ("brake", _float),
        ("steer", _float),        
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = '#MoraiCtrlCmd$'.encode()
        self.data_lenght = 23
        self.aux_data = (0,0,0)
        self.ctrl_mode = 0
        self.gear = 0
        self.cmd_type = 0
        self.velocity = 0
        self.acceleration = 0
        self.accel = 0
        self.brake = 0
        self.steer = 0        
        self.tail = '\r\n'.encode()  
