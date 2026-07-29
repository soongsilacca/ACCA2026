from lib.define.type import *
from lib.define.base import Base

class SetScenarioLoad(Base):
    _fields_ = [
        ("header", _char * 14),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("filename", _char * 30),
        ("delete_all", _bool),
        ("network", _bool),
        ("ego", _bool),
        ("npc", _bool),
        ("pedestrian", _bool),
        ("object", _bool),
        ("pause", _bool),
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = '#ScenarioLoad$'.encode()
        self.data_lenght = 37
        self.aux_data = (0,0,0)
        self.filename = b''
        self.delete_all = False
        self.network = False
        self.ego = False
        self.npc = False
        self.pedestrian = False
        self.object = False
        self.pause = False
        self.tail = '\r\n'.encode()  
