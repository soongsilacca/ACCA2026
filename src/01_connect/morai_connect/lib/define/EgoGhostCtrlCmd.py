from lib.define.type import *
from lib.define.base import Base

class EgoGhostCtrlCmd(Base):
    _fields_ = [
        ("header", _char * 13),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("pose_x", _float),
        ("pose_y", _float),
        ("pose_z", _float),
        ("roll", _float),
        ("pitch", _float),
        ("yaw", _float),
        ("velocity", _float),
        ("steer", _float),
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = '#EgoGhostCmd$'.encode()
        self.data_lenght = 32
        self.aux_data = (0,0,0)
        self.pose_x = 0
        self.pose_y = 0
        self.pose_z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.velocity = 0
        self.steer = 0
        self.tail = '\r\n'.encode()  
