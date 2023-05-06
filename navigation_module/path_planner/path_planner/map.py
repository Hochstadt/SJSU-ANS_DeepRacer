from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import math

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

        self.map_data = grid_map
        self.width = grid_map.info.width
        self.height = grid_map.info.height
        self.resolution = grid_map.info.resolution

        self.origin = Point()
        self.origin.x = grid_map.info.origin.position.x
        self.origin.y = grid_map.info.origin.position.y

    def get_by_index(self, i, j):
        if not self.are_indices_in_range(i, j):
            return 100
        return self.map_data.data[i*self.width + j]

    # i is for row (y), j is for col (x)
    def get_by_coord(self, x, y):
        return self.get_by_index(*self.coord_to_indices(x, y))

    def coord_to_indices(self, x, y):
        i = int((y - self.origin.y) / self.resolution)
        j = int((x - self.origin.x) / self.resolution)
        return (i, j)

    def are_indices_in_range(self, i, j):
        return 0 <= i < self.height and 0 <= j < self.width

    def is_allowed(self, state, robot):
        was_error = False
        i, j = self.coord_to_indices(state.x, state.y)
        side = max(int(math.floor((max(robot.width, robot.height) / self.resolution) / 2)),1)
        try:
            for s_i in range(i-side, i+side):
                for s_j in range(j-side, j+side):
                    cell = self.get_by_index(s_i, s_j)
                    if cell == 100 or cell == -1:
                        return False
        #except IndexError as e:
        except:
            was_error = True
        return True and not was_error