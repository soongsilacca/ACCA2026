from lib.define.type import *
from lib.define.base import Base

class GetTrafficLightStatus(Base):
    _fields_ = [
        ("header", _char * 14),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("trafficLightIndex", _char * 12),
        ("trafficLightType", _short),
        ("trafficLightStatus", _short),
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = b''
        self.data_lenght = 0
        self.aux_data = (0,0,0)
        self.trafficLightIndex = b''
        self.trafficLightType = 0
        self.trafficLightStatus = 0