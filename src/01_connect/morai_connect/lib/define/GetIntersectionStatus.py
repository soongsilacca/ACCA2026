from lib.define.type import *
from lib.define.base import Base

class GetIntersectionStatus(Base):
    _fields_ = [
        ("header", _char * 11),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("intindex", _short),
        ("intstatus", _short),
        ("inttime", _float),
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = b''
        self.data_lenght = 0
        self.aux_data = (0,0,0)
        self.intindex = 0
        self.intstatus = 0
        self.inttime = 0