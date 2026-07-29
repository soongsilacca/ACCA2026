from lib.define.type import *
from lib.define.base import Base

class SensorControl(Base):
    _fields_ = [
        ("header", _char * 15),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("sensorIndex", _int16),
        ("pose_x", _float),
        ("pose_y", _float),
        ("pose_z", _float),
        ("roll", _float),
        ("pitch", _float),
        ("yaw", _float),
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = '#SensorControl$'.encode()
        self.data_lenght = 26
        self.aux_data = (0,0,0)
        self.sensorIndex = 0
        self.pose_x = 0
        self.pose_y = 0
        self.pose_z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.tail = '\r\n'.encode()  
