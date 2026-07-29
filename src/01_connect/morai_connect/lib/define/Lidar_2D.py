import numpy as np
import matplotlib.pyplot as plt        

from lib.define.type import *
from lib.define.base import Base
from matplotlib.animation import FuncAnimation

class RANGE(Base):
    _fields_ = [
        ("distance", _uint16),
        ("intensity", _byte)
    ]

class MORAILIDAR(Base):
    _fields_ = [
        ("header", _char * 9), 
        ("data_length", _int),
        ("aux_data", _float * 3),
        ("payload", (RANGE*360)),
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.range = 360

class LIDAR_UTILS:
    def __init__(self):                
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_aspect('equal')
        self.scatter = self.ax.scatter([], [], s=3)
        self.range = None

    def setData(self, data):
        self.lidar_data = data
        
        if self.range == None:
            self.range = self.lidar_data.get_data().range
            self.angles = np.linspace(0, 2 * np.pi, self.range)
            self.angles = self.angles + np.pi / 2

    def getData(self):
        return self.lidar_data.get_data()

    def visualization(self):
        ani = FuncAnimation(self.fig, self.update, frames=range(100), blit=True, interval=100)
        while 1:
            plt.show()

    def update(self, frame):
        lidar_data = self.getData()                
        try:
            distances = [lidar_data.payload[i].distance/1000 for i in range(self.range)]
        except:
            distances = [0 for i in range(self.range)]        
        finally:
            for i in range(self.range):
                if distances[i] == 10 :
                    distances[i] = 0 
                    

        x = distances * np.cos(self.angles)  # x 좌표로 변환
        y = distances * np.sin(self.angles)  # y 좌표로 변환
        
        self.scatter.set_offsets(np.c_[x, y])  # 데이터 업데이트
        return self.scatter,