######################################################
# Based on code from the following git repo:         #
# https://github.com/LetsPlayNow/TrajectoryPlanner   #
#                                                    #
# Functionality is larger the same but with          #
# improvements and refactor to works as ROS2 node    #
# for Foxy and Humble                                #
###################################################### 

rom nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation as R
import math
import numpy as np

class Map(object):
    def __init__(self, grid_map=None):
        if grid_map is not None:
            self.map_data = grid_map
            self.width = grid_map.info.width
            self.height = grid_map.info.height
            self.resolution = grid_map.info.resolution

            self.origin = Point()
            self.origin.x = grid_map.info.origin.position.x
            self.origin.y = grid_map.info.origin.position.y
        else:
            self.map_data = []
            self.width = 0
            self.height = 0
            self.resolution = 10000000

            self.origin = Point()
            self.origin.x = 0.0
            self.origin.y = 0.0

    def loadMap(self, grid_map):
        # Store map data and dimensions
        self.map_data = grid_map
        self.width = grid_map.info.width
        self.height = grid_map.info.height
        self.resolution = grid_map.info.resolution

        # Store map origin
        self.origin = Point()
        self.origin.x = grid_map.info.origin.position.x
        self.origin.y = grid_map.info.origin.position.y

    def get_by_index(self, i, j):
        # Return occupied if outside of map boundaries
        if not self.are_indices_in_range(i, j):
            return 100

        # Return map value
        return self.map_data.data[i*self.width + j]

    def coord_to_indices(self, x, y):
        # Convert x,y to pixel lcoation
        i = ((y - self.origin.y) / self.resolution)
        j = ((x - self.origin.x) / self.resolution)
        return (i, j)

    def are_indices_in_range(self, i, j):
        # Compare ij to image boundaries
        return 0 <= i < self.height and 0 <= j < self.width

    def is_allowed(self, state, robot):
        was_error = False

        # Calculate robot corner distance from center
        corners = np.array( [[robot.height/2, robot.height/2, -robot.height/2, -robot.height/2],
                             [robot.width/2, -robot.width/2,  -robot.width/2,   robot.width/2]])
        
        # Calculate DCM between robot and map
        rot = R.from_euler('xyz',[0,0,state.theta])
        dcm = np.array(rot.as_matrix())
        
        # Loop through each robot corner and convert to pixel space
        i_corner = np.zeros((4,1))
        j_corner = np.zeros((4,1))
        for corner in range(0,4):
            state_edge = np.matmul(dcm[0:2,0:2], corners[:,corner]) + np.array([state.x, state.y])
            i_corner[corner], j_corner[corner] = self.coord_to_indices(state_edge[0], state_edge[1])

        # Find max and min pixel location in i and j
        i_max = int(np.ceil(np.amax(i_corner)))
        i_min = int(np.floor(np.min(i_corner)))
        j_max = int(np.ceil(np.amax(j_corner)))
        j_min = int(np.floor(np.min(j_corner)))

        # Loop through each ij combo and check if occupied
        try:
            for s_i in range(i_min, i_max+1):
                for s_j in range(j_min, j_max+1):
                    cell = self.get_by_index(s_i, s_j)
                    if cell == 100 or cell == -1:
                        return False
        # Flag any errors
        except:
            was_error = True

        return True and not was_error