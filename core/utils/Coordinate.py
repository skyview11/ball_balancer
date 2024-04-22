import numpy as np

class Coordinate:
    
    coordinate_systems = {}
    
    def __init__(self, init_coordinate, coordinate_system):
        self.coord = np.array(init_coordinate, dtype=np.float32)
        #Coordinate.addNewCoordSystem(coordinate_system)
        self.x = init_coordinate[0]
        self.y = init_coordinate[1]
        self.z = init_coordinate[2]
    
    @staticmethod
    def addNewCoordSystem(coordinate_system):
        if coordinate_system not in Coordinate.coordinate_systems:
            Coordinate.coordinate_systems[coordinate_system] = 1
        else:
            Coordinate.coordinate_systems[coordinate_system] += 1
    
    def __call__(self):
        return self.coord
    
    def __iter__(self):
        return iter(self.coord)
    
    
        
            
        
    