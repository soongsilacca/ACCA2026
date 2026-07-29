from lib.define.type import *
from lib.define.base import Base

class IMU(Base):
    _fields_ = [
        ("header", _char * 9),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("sec", _int),
        ("nsec", _int),
        ("ori_w", _double),
        ("ori_x", _double),
        ("ori_y", _double),
        ("ori_z", _double),
        ("ang_vel_x", _double),
        ("ang_vel_y", _double),
        ("ang_vel_z", _double),
        ("lin_acc_x", _double),
        ("lin_acc_y", _double),
        ("lin_acc_z", _double),
        ("tail", _char * 2)
    ]