from lib.define.type import *
from lib.define.base import Base

class GPS(Base):
    _fields_ = [
        ("header", _char * 6),
        ("data", _char * 1022)
    ]

    def __init__(self):
        self.gprmc = GPRMC()
        self.gpgga = GPGGA()

    def parsing(self): 
        if self.header.decode() == '$GPRMC':
            self.gprmc.parsing(self.data)
            
        elif self.header.decode() == '$GPGGA':
            self.gpgga.parsing(self.data)
        

class GPRMC:
    #https://ko.wikipedia.org/wiki/NMEA_0183
    def __init__(self):        
        self.utc = None
        self.posStatus = None
        self.lat = 0.0
        self.lat_dir = None
        self.lon = 0.0
        self.lon_dir = None
        self.speed = None 
        self.track_true = None
        self.date = None
        self.mag_var = None
        self.var_dir = None
        self.mode_ind = None
        self.check_sum = None
        self.tail = None
        
    def parsing(self, raw_data):
        split_data = raw_data.decode().split(',')
        self.utc = split_data[1]
        self.posStatus = split_data[2]
        self.lat = int(float(split_data[3])/100)+(float(split_data[3])%100)/60        
        self.lat_dir = split_data[4]
        self.lon = int(float(split_data[5])/100)+(float(split_data[5])%100)/60
        self.lon_dir = split_data[6]
        self.speed = split_data[7]
        self.track_true = split_data[8]
        self.date = split_data[9]
        self.mag_var = split_data[10]
        self.var_dir = split_data[11]
        self.mode_ind = split_data[12]
        self.check_sum = split_data[13]


class GPGGA:
    def __init__(self):        
        self.utc = None        
        self.lat = 0.0
        self.lat_dir = None
        self.lon = 0.0
        self.lon_dir = None
        self.quality = None
        self.sats = None
        self.hdop = None
        self.alt = 0.0
        self.a_units = None
        self.undulation = None
        self.u_units = None
        self.age = None
        self.stn_ID = None
        self.check_sum = None        
        self.tail = None
        
    def parsing(self, raw_data):
        split_data = raw_data.decode().split(',')
        # print(split_data)
        try:
            self.utc = split_data[1]        
            self.lat = int(float(split_data[2])/100)+(float(split_data[2])%100)/60        
            self.lat_dir = split_data[3]
            self.lon = int(float(split_data[4])/100)+(float(split_data[4])%100)/60
            self.lon_dir = split_data[5]
            self.quality = split_data[6]
            self.sats = split_data[7]
            self.hdop = split_data[8]
            self.alt = float(split_data[9])
            self.a_units = split_data[10]
            self.undulation = split_data[11]
            self.u_units = split_data[12]
            self.age = split_data[13]
            self.stn_ID = split_data[14]
            self.check_sum = split_data[15]
        except :
            pass
