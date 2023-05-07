import copy
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from path_planner.map import Map
from path_planner.robot import Robot
from rclpy.clock import Clock
import math

class State:

    occMap = Map()

    def __init__(self, x=0.0, y=0.0, theta=0.0, parent=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.parent = parent

        # Variables for A* algorhytm
        self.g = self.f = self.h = 0

    @staticmethod
    def from_pose(pose):

        # Initialize State
        new_state = State()

        # Store position
        new_state.x = pose.position.x
        new_state.y = pose.position.y

        # Calculate rotation Angles
        rot = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        angles = rot.as_euler('xyz')
        roll = angles[0]
        pitch = angles[1]
        yaw = angles[2]

        # Store yaw
        new_state.theta = yaw
        
        return new_state

    def dist_to(self, o_s):
        return ((self.x - o_s.x)**2 + (self.y - o_s.y)**2)**0.5

    def is_same_as(self, o_s):
        # Improoving this constant from 0.01 to 0.05 speed up my algorhytm 1000 times
        return self.dist_to(o_s) <= 0.05 # think a much better about this constant

    def apply(self, move):
        new_theta = self.theta + move.dtheta
        new_x = self.x + math.cos(new_theta) * move.length
        new_y = self.y + math.sin(new_theta) * move.length
        return State(new_x, new_y, new_theta)

    def try_apply(self, _map, move, robot):
        model_state = copy.copy(self)
        model_state.parent = self
        self.occMap = _map

        # TODO fix this misunderstanding with goal
        goal = self.apply(move)
        goal.parent = self
        steps_count = max(int(self.dist_to(goal) / min(robot.height, robot.width)), 1)
        step = 1.0 / steps_count

        for i in range(steps_count):
            # Slightly change angle
            model_state.theta += move.dtheta * step

            # Go in this direction
            model_state.x += math.cos(model_state.theta) * move.length * step
            model_state.y += math.sin(model_state.theta) * move.length * step

            if not self.occMap.is_allowed(model_state, robot):
                return None
        return goal

    def to_pose_stamped(self):

        pose = PoseStamped()
        pose.header.frame_id = "odom"
        time_stamp = Clock().now()
        pose.header.stamp = time_stamp.to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0

        rot = R.from_euler('xyz', [0, 0, self.theta])
        quaternion = rot.as_quat()
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        return pose