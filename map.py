import numpy as np


"""
 This file is a map object that will hold our complete map and update it based on lidar scans
 The cells are held within a map to allow for dynamic expansion of map
"""
class Map:
    def __init__(self, resolution):
        self.resolution = resolution
        self.res_place = 100
        self.max_range = 10
        self.obstacles = {}

    ############## Public member functions ##################
    # takes in point and finds cell that contains the point -- returns if there is obstacle there or not
    def isObstacle(self, x, y):
        pass

    def updateFromLidar(self, ):
        pass
    
    # Resets map
    def clear_map(self):
        self.obstacles.clear()

    def set_resolution(self, resolution):
        self.resolution = resolution

    ############ Private member functions ###############
    # Use cantor pairing to create hash for coordinate
    def hash_coord(self, x, y):
        # can use the fact that our resolution is to the 100ths place
        x_coord = x * 100
        y_coord = y * 100

        cantor_pair = (0.5) * (x_coord + y_coord) * (x_coord + y_coord + 1) + y_coord
        return cantor_pair

