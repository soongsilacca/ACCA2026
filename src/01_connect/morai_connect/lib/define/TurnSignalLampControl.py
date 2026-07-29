from lib.define.type import *
from lib.define.base import Base

class TurnSignalLampControl(Base):
    _fields_ = [
        ("header", _char * 13),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("turnsignal", _byte),
        ("emergencySignal", _byte),        
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = '#LampControl$'.encode()
        self.data_lenght = 2
        self.aux_data = (0,0,0)
        self.turnsignal = 0
        self.emergencySignal = 0
        self.tail = '\r\n'.encode()  
