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
        self.newObstacles = []

    ############## Public member functions ##################
    # takes in point and finds cell that contains the point -- returns if there is obstacle there or not
    def isObstacle(self, point):
        hash = self.hash_coord(point)
        if self.obstacles.get(hash) is None:
            return False
        else:
            return True

    # Updates global map and gives new obstacles that were added
    def updateMap(self, obstacle_points):
        # Clear new obstacles list from last update
        self.newObstacles.clear()
        # Iterate through each point seen by lidar
        for point in obstacle_points:
            # Hash the point
            hash = self.hash_coord(point)

            if self.obstacles.get(hash) is None:
                self.newObstacles.append(point)
            self.obstacles[hash] = True

    def getNewPoints(self):
        return self.newObstacles


    # Resets map
    def clear_map(self):
        self.obstacles.clear()

    def set_resolution(self, resolution):
        self.resolution = resolution

    ############ Private member functions ###############
    # Use cantor pairing to create hash for coordinate
    def hash_coord(self, point):
        # convert to integer
        x_coord = point[0] * 100
        y_coord = point[1] * 100

        # convert to positive integer
        x_coord += (self.max_range / self.resolution)
        y_coord += (self.max_range / self.resolution)

        cantor_pair = (0.5) * (x_coord + y_coord) * (x_coord + y_coord + 1) + y_coord
        return cantor_pair

