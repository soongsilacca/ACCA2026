from lib.define.type import *
from lib.define.base import Base

class SetIntersectionStatus(Base):
    _fields_ = [
        ("header", _char * 14),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("IntIndex", _short),        
        ("IntStatus", _short),
        ("IntTime", _float),
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = '#SetIntStatus$'.encode()
        self.data_lenght = 8
        self.aux_data = (0,0,0)
        self.IntIndex = 0
        self.IntStatus = 0
        self.IntTime = 0
        self.tail = '\r\n'.encode()  
