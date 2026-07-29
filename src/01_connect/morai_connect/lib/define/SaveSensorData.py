from lib.define.type import *
from lib.define.base import Base

class SaveSensorData(Base):
    _fields_ = [
        ("header", _char * 16),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("custom", _bool),
        ("file_name", _char * 30),
        ("file_dir", _char * 60),
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = '#SaveSensorData$'.encode()
        self.data_lenght = 91
        self.aux_data = (0,0,0)
        self.custom = False
        self.file_name = b''
        self.file_dir = b''
        self.tail = '\r\n'.encode()  
