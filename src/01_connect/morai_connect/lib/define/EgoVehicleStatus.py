from lib.define.type import *
from lib.define.base import Base

class EgoVehicleStatus(Base):
    _fields_ = [
        ("header", _char * 11),
        ("data_lenght", _int),
        ("aux_data", _int * 3),
        ("sec", _int),
        ("nsec", _int),
        ("ctrl_mode", _byte),
        ("gear", _byte),
        ("signed_vel", _float),
        ("map_data_id", _int),
        ("accel", _float),
        ("brake", _float),
        ("size_x", _float),
        ("size_y", _float),
        ("size_z", _float),
        ("overhang", _float),
        ("wheelbase", _float),
        ("rear_overhang", _float),
        ("pos_x", _float),
        ("pos_y", _float),
        ("pos_z", _float),
        ("roll", _float),
        ("pitch", _float),
        ("yaw", _float),
        ("vel_x", _float),
        ("vel_y", _float),
        ("vel_z", _float),
        ("ang_vel_x", _float),
        ("ang_vel_y", _float),
        ("ang_vel_z", _float),
        ("accel_x", _float),
        ("accel_y", _float),
        ("accel_z", _float),
        ("steer", _float),
        ("link_id", _char * 38),
        ("tire_lateral_force_fl", _float), 
        ("tire_lateral_force_fr", _float), 
        ("tire_lateral_force_rl", _float), 
        ("tire_lateral_force_rr", _float), 
        ("side_slip_angle_fl", _float), 
        ("side_slip_angle_fr", _float), 
        ("side_slip_angle_rl", _float), 
        ("side_slip_angle_rr", _float), 
        ("tire_cornering_stiffness_fl", _float), 
        ("tire_cornering_stiffness_fr", _float), 
        ("tire_cornering_stiffness_rl", _float), 
        ("tire_cornering_stiffness_rr", _float), 
        ("tail", _char * 2)

    ]

    def __init__(self):
        self.header = b''
        self.data_lenght = 0
        self.aux_data = (0,0,0)
        self.sec = 0
        self.nsec = 0
        self.ctrl_mode = 0
        self.gear = 0
        self.signed_vel = 0
        self.map_data_id = 0
        self.accel = 0
        self.brake = 0
        self.size_x = 0
        self.size_y = 0
        self.size_z = 0
        self.overhang = 0 
        self.wheelbase = 0
        self.rear_overhang = 0
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
        self.ang_vel_x = 0
        self.ang_vel_y = 0
        self.ang_vel_z = 0
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.steer = 0
        self.link_id = b'' 
        self.tire_lateral_force_fl = 0
        self.tire_lateral_force_fr = 0
        self.tire_lateral_force_rl = 0
        self.tire_lateral_force_rr = 0
        self.side_slip_angle_fl = 0
        self.side_slip_angle_fr = 0
        self.side_slip_angle_rl = 0
        self.side_slip_angle_ = 0
        self.tire_cornering_stiffness_fl = 0
        self.tire_cornering_stiffness_fr = 0
        self.tire_cornering_stiffness_rl = 0
        self.tire_cornering_stiffness_rr = 0