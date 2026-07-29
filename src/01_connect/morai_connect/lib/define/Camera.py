import cv2
import ctypes
import numpy as np
from lib.define.type import *
from lib.define.base import Base

class Camera(Base):
    _fields_ = [
        ("header", _char * 3),      
        ("data", _byte * 64997),
    ]    

    def __init__(self):        
        self.image = IMAGE()
        self.image_size = ctypes.sizeof(self.image)
        self.buffer = b''

        self.bbox = BBOX()
        self.bbox_size = ctypes.sizeof(self.bbox)
        self.bbox_timestamp = 0.0

        self.utils = UTILS()

    def parsing(self):                
        if self.header.decode() == 'MOR':
            
            ctypes.memmove(ctypes.addressof(self.image), self.data, self.image_size)            

            # print(self.image.tail)
            self.buffer += bytes(self.image.jpeg_data)            
            if self.image.tail.decode() == 'EI':                
                self.image.data = self.buffer   
                self.buffer = b''

        elif self.header.decode() == 'BOX':
            ctypes.memmove(ctypes.addressof(self.bbox), self.data, self.bbox_size)
            self.bbox_timestamp = self.bbox.sec + (self.bbox.nsec/1000000000)
            
    
class IMAGE(Base):
    _fields_ = [
        ("sec", _int),
        ("nsec", _int),
        ("index", _int),
        ("size", _int),
        ("jpeg_data", _byte * 64979),
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.header = b''
        self.sec = 0
        self.nsec = 0
        self.index = 0
        self.size = 0
        self.jpeg_data = (_byte * 64979)()
        self.tail = b''

        self.data = b''

class ObjectData(Base):
    _fields_ = [
        ("_3D_BBOX_X1", _float),
        ("_3D_BBOX_Y1", _float),
        ("_3D_BBOX_Z1", _float),
        ("_3D_BBOX_X2", _float),
        ("_3D_BBOX_Y2", _float),
        ("_3D_BBOX_Z2", _float),
        ("_3D_BBOX_X3", _float),
        ("_3D_BBOX_Y3", _float),
        ("_3D_BBOX_Z3", _float),
        ("_3D_BBOX_X4", _float),
        ("_3D_BBOX_Y4", _float),
        ("_3D_BBOX_Z4", _float),
        ("_3D_BBOX_X5", _float),
        ("_3D_BBOX_Y5", _float),
        ("_3D_BBOX_Z5", _float),
        ("_3D_BBOX_X6", _float),
        ("_3D_BBOX_Y6", _float),
        ("_3D_BBOX_Z6", _float),
        ("_3D_BBOX_X7", _float),
        ("_3D_BBOX_Y7", _float),
        ("_3D_BBOX_Z7", _float),
        ("_3D_BBOX_X8", _float),
        ("_3D_BBOX_Y8", _float),
        ("_3D_BBOX_Z8", _float),
        ("_2D_BBOX_XMIN", _float),
        ("_2D_BBOX_YMIN", _float),
        ("_2D_BBOX_XMAX", _float),
        ("_2D_BBOX_YMAX", _float),
        ("Group", _uint8),
        ("Class", _uint8),
        ("SubClass", _uint8),
    ]
    def __init__(self):
        self._3D_BBOX_Y1 = 0
        self._3D_BBOX_X1 = 0
        self._3D_BBOX_Z1 = 0
        self._3D_BBOX_X2 = 0
        self._3D_BBOX_Y2 = 0
        self._3D_BBOX_Z2 = 0
        self._3D_BBOX_X3 = 0
        self._3D_BBOX_Y3 = 0
        self._3D_BBOX_Z3 = 0
        self._3D_BBOX_X4 = 0
        self._3D_BBOX_Y4 = 0
        self._3D_BBOX_Z4 = 0
        self._3D_BBOX_X5 = 0
        self._3D_BBOX_Y5 = 0
        self._3D_BBOX_Z5 = 0
        self._3D_BBOX_X6 = 0
        self._3D_BBOX_Y6 = 0
        self._3D_BBOX_Z6 = 0
        self._3D_BBOX_X7 = 0
        self._3D_BBOX_Y7 = 0
        self._3D_BBOX_Z7 = 0
        self._3D_BBOX_X8 = 0
        self._3D_BBOX_Y8 = 0
        self._3D_BBOX_Z8 = 0
        self._2D_BBOX_XMIN = 0
        self._2D_BBOX_YMIN = 0
        self._2D_BBOX_XMAX = 0
        self._2D_BBOX_YMAX = 0
        self.Group = 0
        self.Class = 0
        self.SubClass = 0
        

class BBOX(Base):
    _fields_ = [
        ("sec", _int),
        ("nsec", _int),
        ("index", _int),
        ("size", _int),
        ("object_data", ObjectData * 575),
        ("tail", _char * 2)
    ]

    def __init__(self):
        self.sec = 0
        self.nsec = 0
        self.index = 0
        self.size = 0
        self.object_data = (ObjectData * 575)()
        self.tail = b''

        self.data = None

    
class UTILS:
    def setResolution(self, w ,h ,fov):
        self.R = np.eye(3)
        self.T = np.zeros((3, 1))
        self.K_matrix(w,h,fov)

    def K_matrix(self,w,h,fov):
        f = w / (2 * np.tan(np.deg2rad(fov)/2))
        self.K =  np.array([[f, 0, w/2],
                       [0, f, h/2],
                       [0, 0, 1]],dtype=np.float32)

    def projectPoints(self, points):
        
        _point_list = [[points._3D_BBOX_X1, points._3D_BBOX_Y1, points._3D_BBOX_Z1],
                      [points._3D_BBOX_X2, points._3D_BBOX_Y2, points._3D_BBOX_Z2],
                      [points._3D_BBOX_X3, points._3D_BBOX_Y3, points._3D_BBOX_Z3],
                      [points._3D_BBOX_X4, points._3D_BBOX_Y4, points._3D_BBOX_Z4],
                      [points._3D_BBOX_X5, points._3D_BBOX_Y5, points._3D_BBOX_Z5],
                      [points._3D_BBOX_X6, points._3D_BBOX_Y6, points._3D_BBOX_Z6],
                      [points._3D_BBOX_X7, points._3D_BBOX_Y7, points._3D_BBOX_Z7],
                      [points._3D_BBOX_X8, points._3D_BBOX_Y8, points._3D_BBOX_Z8]]
        
        point_list = []
        for _point in _point_list:            
            point_list.append(cv2.projectPoints(np.array([[_point[0]],[_point[1]],[_point[2]]]), self.R, self.T, self.K, None)[0][0][0])
            
        return point_list    

    def draw_3d_bbox(self, img, points, color):        
        cv2.line(img, (int(points[0][0]),int(points[0][1])), (int(points[1][0]), int(points[1][1])), color, 1)
        cv2.line(img, (int(points[0][0]),int(points[0][1])), (int(points[2][0]), int(points[2][1])), color, 1)
        cv2.line(img, (int(points[0][0]),int(points[0][1])), (int(points[4][0]), int(points[4][1])), color, 1)
        cv2.line(img, (int(points[1][0]),int(points[1][1])), (int(points[3][0]), int(points[3][1])), color, 1)
        cv2.line(img, (int(points[1][0]),int(points[1][1])), (int(points[5][0]), int(points[5][1])), color, 1)
        cv2.line(img, (int(points[2][0]),int(points[2][1])), (int(points[3][0]), int(points[3][1])), color, 1)
        cv2.line(img, (int(points[2][0]),int(points[2][1])), (int(points[6][0]), int(points[6][1])), color, 1)
        cv2.line(img, (int(points[3][0]),int(points[3][1])), (int(points[7][0]), int(points[7][1])), color, 1)
        cv2.line(img, (int(points[4][0]),int(points[4][1])), (int(points[5][0]), int(points[5][1])), color, 1)
        cv2.line(img, (int(points[4][0]),int(points[4][1])), (int(points[6][0]), int(points[6][1])), color, 1)
        cv2.line(img, (int(points[5][0]),int(points[5][1])), (int(points[7][0]), int(points[7][1])), color, 1)
        cv2.line(img, (int(points[6][0]),int(points[6][1])), (int(points[7][0]), int(points[7][1])), color, 1)

    def draw_2d_bbox(self,img, point_1, point_2, color):
        cv2.rectangle(img, point_1, point_2, color,1)
