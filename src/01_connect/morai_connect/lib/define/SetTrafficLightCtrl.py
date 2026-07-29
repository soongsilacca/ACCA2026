from lib.define.type import *
from lib.define.base import Base

class SetTrafficLightCtrl(Base):
    _fields_ = [
        ("header", _char * 14),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("trafficLightIndex", _char * 12),
        ("trafficLightStatus", _short),
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = '#TrafficLight$'.encode()
        self.data_lenght = 14
        self.aux_data = (0,0,0)
        self.trafficLightIndex = b''
        self.trafficLightStatus = 0
        self.tail = '\r\n'.encode()  
