######################################################
# Based on code from the following git repo:         #
# https://github.com/LetsPlayNow/TrajectoryPlanner   #
#                                                    #
# Functionality is larger the same but with          #
# improvements and refactor to works as ROS2 node    #
# for Foxy and Humble                                #
###################################################### 

#ROS
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import ros2_numpy

#Python
import os
import math
import threading
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
#from exceptions import IndexError
import heapq
import copy

# Path Planner
from path_planner.move import Move
from path_planner.robot import Robot
from path_planner.map import Map
from path_planner.state import State


class Path_Planner(Node):

    motions   = [0.3, 0.0, 0.3, 1.5708/3, 0.3, -1.5708/3]
    robot_height    = 0.3048
    robot_width     = 0.127

    def __init__(self):
        super().__init__('path_planner')

        self.get_logger().info("Started")

        # Read in configurable parameters
        self.load_config()

        # Establish QoS parameters
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        # Create Map Load message subscriber
        self.mGoalPose = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.store_goal_state,
            qos_profile=qos_profile)

        # Create Map Load message subscriber
        self.mCurrentPose = self.create_subscription(
            PoseStamped,
            "/localization/pose",
            self.store_current_state,
            qos_profile=qos_profile)             
            
        # Create Map Load message subscriber
        self.mOccMap= self.create_subscription(
            OccupancyGrid,
            "/ans_services/occupancy_map_msg",
            self.store_map,
            qos_profile=qos_profile)

        # Create Path Plan request subscriber
        self.mPathPlanRequest = self.create_subscription(
            Bool,
            "/controller/path_request",
            self.indicate_plan_request,
            qos_profile=qos_profile)                 
           
        # Create atGoalState publisher
        self.mPathPub = self.create_publisher(
            Path,
            "/path_planner/path",
            1)

        # Initialize state and map variables
        self.currentState = State()
        self.goalState    = State()
        self.occMap       = Map()

        # Set flag for readiness
        self.waitForPlan = True
        self.mapReady    = False
        self.goalReady   = False

    def load_config(self):

        # Declare input variables
        self.declare_parameter("motions",  self.motions)
        self.declare_parameter("robot_height",  self.robot_height)
        self.declare_parameter("robot_width",  self.robot_width)

        # Set input variables
        self.motions = self.get_parameter("motions").get_parameter_value().double_array_value
        self.robot_height = self.get_parameter("robot_height").get_parameter_value().double_value
        self.robot_width = self.get_parameter("robot_width").get_parameter_value().double_value

        # Construct Moves object
        self.moves = []
        for i in range(0, int(len(self.motions)/2)):
            self.moves.append(Move(self.motions[2*i], self.motions[2*i+1]))
        
        # Construct Robot object
        self.robot = Robot(self.robot_height, self.robot_width)

    def store_map(self, msg):

        # Inform that message was received
        self.get_logger().info('Map has been received')
        
        # convert ros msg to open3d
        self.occMap = Map(msg)

        # Set waitForPlan flag to true
        self.waitForPlan = True
        self.mapReady   = True

    def store_goal_state(self, goalMsg):
            
        # Inform that message was received
        self.get_logger().info("Goal State received")

        # Initialize Goal State Pose
        self.goalState = State.from_pose(goalMsg.pose)

        # Set waitForPlan flag to true
        self.waitForPlan = True
        self.goalReady   = True

    def store_current_state(self, currentMsg):

        # Initialize Goal State Pose
        self.currentState = State.from_pose(currentMsg.pose)

        # If no valid plan exists run planner
        if self.waitForPlan and self.goalReady and self.mapReady:
            self.generate_path_plan()

    def indicate_plan_request(self, msg):

        # Set waitForPlan flag to true
        self.waitForPlan = True

    def generate_path_plan(self):

        self.get_logger().info("Planning was started")

        t0 = time.time()

        # Calculate path if goal is reachable
        if self.occMap.is_allowed(self.goalState, self.robot):
            
            # Execute A* algorithm
            final_state = self.replan(self.goalState)
            current_state = copy.copy(final_state)

            # if valid solution found
            if current_state is not None:

                # Initialize path plan
                path = Path()
                path.header.frame_id = 'odom'
            
                # Loop through each way point and store in path plan
                while True:
                    pose_point = current_state.to_pose_stamped()
                    path.poses.append(pose_point)

                    current_state = current_state.parent

                    if current_state is None:
                        break

                # Publish Path
                self.mPathPub.publish(path)

                # Set waitForPlan flag to false
                self.get_logger().info(f"Path found in {(time.time() - t0)} s")
                self.waitForPlan = False

            # if no valid solution found
            else:
                self.get_logger().error("No path found!")
                self.waitForPlan = False
        
        # Error if goal state is occupied location
        else:
            self.get_logger().error("Goal State is in occupied location")
            self.waitForPlan = False
            

    def replan(self, goal):

        # Initialize final state and queues
        final_state = None
        opened = []
        closed = []

        # Add starting state to open queue
        heapq.heappush(opened, (0.0, self.currentState))

        # Continue until goal reached
        while opened and final_state is None:

            # pop state from open queue
            q = heapq.heappop(opened)[1]

            # loop through each possible movement
            for move in self.moves:

                # Attempt to execute movement
                successor = q.try_apply(self.occMap, move, self.robot)
                
                # Proceed if movement is allowed
                if successor is not None:

                    # Stop if goal state is achieved
                    
                    if successor.dist_to(goal) < max(self.moves[0].length, min(self.robot.width, self.robot.height)):
                        final_state = successor
                        return final_state
                        break

                    # Calculate distances and heuristic
                    successor.g = q.g + successor.dist_to(q)
                    successor.h = successor.dist_to(goal)
                    successor.f = successor.g + successor.h
                    successor.parent = q

                    # Check if movement is better than any current movement in open list
                    better_in_opened = False
                    for other_f, other_successor in opened:
                        if other_successor.is_same_as(successor) and other_f <= successor.f:
                            better_in_opened = True

                    # Check if movement is better than any current movment in closed list
                    if not better_in_opened:
                        better_in_closed = False
                        for other_successor in closed:
                            if other_successor.is_same_as(successor) and other_successor.f <= successor.f:
                                better_in_closed = True

                        # If all conditions met add to open list
                        if not better_in_closed:
                            heapq.heappush(opened, (successor.f, successor))

            # Add q to closed list
            closed.append(q)
        
        # Return final path plan
        #final_state = successor
        return final_state
        

def main(args=None):
    rclpy.init(args=args)
    
    path_planner = Path_Planner()
    print('Spinning Node...')
    rclpy.spin(path_planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print('Destroying Node')
    path_planner.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
