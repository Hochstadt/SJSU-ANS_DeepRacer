######################################################
# Based on code from the following git repo:         #
# https://github.com/LetsPlayNow/TrajectoryPlanner   #
#                                                    #
# Functionality is larger the same but with          #
# improvements and refactor to works as ROS2 node    #
# for Foxy and Humble                                #
###################################################### 

class Robot:
    def __init__(self, height = 2.0, width = 3.0):
        self.width = width
        self.height = height