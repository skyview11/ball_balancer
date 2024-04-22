import math
import random
from utils import Coordinate
from utils.functions import *
import numpy as np




class MotorDegreeCalculator:
    def __init__(self):
        self.base_centroid_to_vertex = 58.737
        self.platform_centroid_to_vertex = 110.737
        self.conn1_len = 52
        self.conn2_len = 60
        self.setTargetHeight(self.conn2_len)
        self.max_angle = toRadian(15)
        
        self.targetNormal = Coordinate((0, 0, 1), "world")
        self.platform_vertices = np.array([[self.platform_centroid_to_vertex, 0, 0], \
                                  [-0.5 * self.platform_centroid_to_vertex, math.sqrt(3)/2*self.platform_centroid_to_vertex, 0], \
                                  [-0.5 * self.platform_centroid_to_vertex, -math.sqrt(3)/2*self.platform_centroid_to_vertex, 0]
                                  ]).T
        self.base_vertices = np.array([[self.base_centroid_to_vertex, 0, 0], \
                                  [-0.5 * self.base_centroid_to_vertex, math.sqrt(3)/2*self.base_centroid_to_vertex, 0], \
                                  [-0.5 * self.base_centroid_to_vertex, -math.sqrt(3)/2*self.base_centroid_to_vertex, 0]
                                  ]).T
        
        # flags and buffer to reduce the operation
        self.__isCaculated = 0
        self.__caculated_motor_angle = -1
        
        
        # debug
        self.failed = 0
        
        
    def setTargetNormalVector(self, vector):
        if getAngleFromNormal(vector) <= self.max_angle:
            norm = getNorm(vector)
            self.targetNormal = Coordinate([vector[0]/norm, -vector[1]/norm, vector[2]/norm], 'world')
            self.__isCaculated = 0
        elif vector == float('NaN'):
            self.targetNormal = Coordinate([0,0,1], "world")
        else:
            #self.targetNormal = self.max_angle
            new_z = math.tan(math.pi/2 - (self.max_angle - toRadian(1))) * getNorm((vector[:2]))
            new_vec = (vector[0], vector[1], new_z)
            print(new_vec)
            norm = getNorm(new_vec)
            
            self.targetNormal = Coordinate([new_vec[0]/norm, -new_vec[1]/norm, new_vec[2]/norm], 'world')
            self.__isCaculated = 0
            
            if getAngleFromNormal(new_vec) >self.max_angle:
                import pdb;pdb.set_trace()
            f"Angle of the normal vector is {getAngleFromNormal(new_vec) * 180/math.pi}, max angle is too big"
            
    def setTargetHeight(self, height):
        self.height = height
        
        self.height_mtx = np.tile(np.transpose(np.array([0, 0, self.height]).reshape(1, -1)), (1, 3))
        
    
    def calculateMotorAngles(self, roll=-1, pitch=-1):
        
        
        
        
        
        if (self.__isCaculated==0):
            if (roll==-1 and pitch==-1):
                roll, pitch = getRollPitchFromNormalVector(self.targetNormal.coord)
            rotation_matrix = getRotationMatrixFomRollPitch(roll, pitch)
            platform_world_coords = (rotation_matrix @ self.platform_vertices) + self.height_mtx
            
            
            # 플랫폼의 타깃 좌표와 모터의 중심 좌표로부터 삼각형을 구하고, 구해진 삼각형으로부터 제2코사인 법칙으로 각을 구함. 
            conn_mtx = platform_world_coords - self.base_vertices
            sides = [[self.conn2_len, self.conn1_len, getNorm(list(conn_mtx[:,i].T))] for i in range(3)]
            
            try:
                self.caculated_motor_angle = [math.pi/2-cos2Law(side)- math.atan(abs(getNorm(conn_mtx[:2, i].T)/conn_mtx[2, i])) for i, side in enumerate(sides)]
            except Exception as e:
                self.caculated_motor_angle = (0,0,0)
                print(e)
                
            self.__isCaculated = 1
        
        return self.caculated_motor_angle
        
if __name__ == '__main__':
    from tqdm import tqdm
    import time
    calculator = MotorDegreeCalculator()
    n = 100
    t = 0
    for _ in tqdm(range(n)):
        vector = [random.random() for _ in range(3)]
        vector[2] = abs(vector[2])
       
        while getAngleFromNormal(vector) > toRadian(5):
            vector = [random.random() for _ in range(3)]
            vector[2] = abs(vector[2])
        t0 = time.time()
        
        calculator.setTargetNormalVector(vector)
        a = calculator.calculateMotorAngles()
        print(a)
        t += (time.time()-t0)
        

    print(getAngleFromNormal(vector)*180/math.pi, [i*180/math.pi for i in a])
        