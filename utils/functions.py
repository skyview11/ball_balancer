import math
from .Coordinate import Coordinate
import numpy as np

def toRadian(angle):
    return angle * (math.pi/180)
def getNorm(vector):
    return sum([(elem)**2 for elem in vector])**0.5
def getAngleFromNormal(vector):
    return math.asin(getNorm((vector[0], vector[1], 0))/getNorm(vector))

def getRollPitchFromNormalVector(cart_coord:Coordinate):
    pitch = math.asin(cart_coord[0])
    roll = math.atan2(-cart_coord[1], cart_coord[2])
    return roll, pitch
    
def getRotationMatrixFomRollPitch(roll, pitch):
    cos_roll = math.cos(roll)
    sin_roll = math.sin(roll)
    cos_pit = math.cos(pitch)
    sin_pit = math.sin(pitch)
    
    
    R_x = np.array([[1, 0, 0], 
                    [0, cos_roll, -sin_roll], 
                    [0, sin_roll, cos_roll]])
    R_y = np.array([[cos_pit, 0, sin_pit], 
                    [0, 1, 0], 
                    [-sin_pit, 0, cos_pit]])
    return R_x @ R_y
def cos2Law(sides):
    """_summary_

    Args:
        sides (list): first elem should be hypotenuse
    """
   
    assert (2*max(sides) < sum(sides)), "Not triangle!"
    
    hypotenuse, side_a, side_b = sides
    return math.acos((side_a**2 + side_b**2 - hypotenuse**2) / (2 * side_a * side_b))
    