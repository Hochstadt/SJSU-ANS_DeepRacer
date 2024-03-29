import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from std_msgs.msg import Header
from nav_msgs.msg import Path



from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


#Python
import os
import pickle
import numpy as np
from time import sleep
from datetime import datetime
from scipy.spatial.transform import Rotation as R



class navigatorHost(Node):
    def __init__(self):
        super().__init__('navigator_host')
        self.bMapLoaded = False
        map_file = 'na'
        occ_file = 'na'

        #Expecting this to be the full path to the map file. If na will search
        #the current directory for one
        #self.declare_parameter('map_file', map_file)
        #Only add this if needed....
        #self.declare_parameter('occ_file', occ_file)

        #self.declare_parameter('goal_state', goal_state)
        #reports the aligned poitn cloud
        self.debug_point_array = -1
        #reports the pose
        self.point = -1
        self.quat = -1

        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
                )
        
        self.pose_subscriber = self.create_subscription(PoseStamped,
                                                        '/localization/pose',
                                                        self.pose_listener, 
                                                        qos_profile=qos_profile)

        self.solution_found_subscriber = self.create_subscription(Bool,
                                                        '/localization/solution_found',
                                                        self.solution_found_listener, 
                                                        qos_profile=qos_profile)
        

        self.debug_point_cloud = self.create_subscription(PointCloud2,
                                                        '/localization/aligned_lidar_pt_msg',
                                                        self.pt_cloud_listener, 
                                                        qos_profile=qos_profile)

        #self.map_file = self.get_parameter('map_file').get_parameter_value().string_value
        
    def path_listener(self, msg):
        self.get_logger().info('Received the path')

    def pose_listener(self, msg):
        self.point = msg.pose.position
        self.quat = msg.pose.orientation

    def pt_cloud_listener(self, msg):
        point_list = point_cloud2.read_points_list(msg)
        pl_length = len(point_list)
        point_array = np.empty((3, pl_length))
        for i, p in enumerate(point_list):
            point_array[0, i] = p.x
            point_array[1,i] = p.y
            point_array[2,i] = p.z
        self.debug_point_array = point_array

    def solution_found_listener(self, msg):
        bSolutionFound = msg.data
        if bSolutionFound == True:
            #self.get_logger().info('Localization Successful')
            #self.get_logger().info('Position: <%.4f %.4f %.4f>' % (self.point.x, self.point.y, self.point.z))
            #self.get_logger().info('Quaternion: <%.4f %.4f %.4f, %.4f>' % (self.quat.x, self.quat.y, self.quat.z, self.quat.w))

            #Debugging info
            with open('debug_map.pickle', 'wb') as handle:
                pickle.dump(self.debug_point_array, handle)

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]
    
    def quat_to_theta(self, q):
        r = R.from_quat([q.x, q.y, q.z, q.w])
        e = r.as_euler('zyx')
        return e[0]
    
    def get_hand_path(self, starting_pos, orientation):

        goal = np.array([[3.75, 0.2]]).transpose()
        #hand_path = np.array([[4.5, -2.5], [5, -2], [5, -1]]).transpose()
        hand_path = np.array([[4.5, -2.5],[5, -1.5], [5,0], [4.25, 0.2]]).transpose()
        hand_path = np.hstack((starting_pos, hand_path))
        hand_path = np.hstack((hand_path, goal))
        sample_num = 2
        total_pts= hand_path



        '''
        total_pts = np.array([])
        for i in range(0, hand_path.shape[1]-1):
            st_x = hand_path[0, i]
            st_y = hand_path[1, i]
            ed_x = hand_path[0, i + 1]
            ed_y = hand_path[1, i + 1]

            #define this vector
            #vect = np.array([ed_x - st_x,ed_y - st_y])
            #x_unit = np.array([1, 0])
            #rotangle = np.arccos(np.dot(vect, x_unit)/(np.linalg.norm(x_unit) * np.linalg.norm(vect)))
            
            pts_x = np.linspace(st_x, ed_x, num=sample_num)
            pts_y = np.linspace(st_y, ed_y, num=sample_num)
            if np.any(total_pts):
                total_pts = np.hstack((total_pts, np.array([pts_x[0:-1], pts_y[0:-1]])))
                #rots = np.linspace(total_rots[-1], rotangle, num=sample_num)
                #total_rots = np.hstack((total_rots, np.ones(sample_num)*rotangle))
                
            else:
                #rots = np.linspace(orientation, rotangle, num=sample_num)
                total_pts = np.array([pts_x[0:-1], pts_y[0:-1]])
                #total_rots = rots
        '''
        #Run through and define the tangent for each path starting with the next index
        total_rots = [orientation]
        for i in range(1, total_pts.shape[1] - 1):
            pt_x1 = total_pts[0, i-1]
            pt_y1 = total_pts[1, i-1]
            pt_x2 = total_pts[0, i]
            pt_y2 = total_pts[1, i]

            dx = pt_x2 - pt_x1
            dy = pt_y2 - pt_y1
            self.get_logger().info('pt_x2: %.4f, pt_x1 %.4f, dx %.4f' % (pt_x2, pt_x1, dx))
            self.get_logger().info('pt_y2: %.4f, pt_y1 %.4f, dy %.4f' % (pt_y2, pt_y1, dy))
            angle = np.arctan(dy/dx)
            if angle < 0:
                angle += np.pi
            total_rots.append(angle)
            self.get_logger().info('Rotation = %.4f' % np.rad2deg(angle))
        #Hopefully this is tangenet enough to do this
        total_rots.append(total_rots[-1])
        total_rots = np.array(total_rots)
        self.get_logger().info('Rotation size %d' % total_rots.shape[0])
        #Now convert to the appropriate parameters
        epoch = datetime.now()
        myHeader = Header()
        myHeader.stamp.sec = 0
        myHeader.stamp.nanosec = 0
        myHeader.frame_id = "odom"
        myPath = Path()
        myPath.header = myHeader
        for i in range(0, total_pts.shape[1]):
            pt = total_pts[:, i]
            yaw = total_rots[i]
            Q = Quaternion()
            tmpq = self.get_quaternion_from_euler(0, 0, yaw)
            Q.x = tmpq[0]
            Q.y = tmpq[1]
            Q.z = tmpq[2]
            Q.w = tmpq[3]

            P = Point()
            P.x = total_pts[0,i]
            P.y = total_pts[1,i]
            tmpPose = Pose()
            tmpPose.position = P
            tmpPose.orientation = Q
            myHeader = Header()
            myHeader.stamp.sec = int(np.floor((datetime.now() - epoch).total_seconds()))
            #Using miliseconds as nanoseconds
            myHeader.stamp.nanosec = int(np.floor((datetime.now() - epoch).total_seconds() * 10**6))
            myHeader.frame_id = "odom"
            tmpPS = PoseStamped()
            tmpPS.header = myHeader
            tmpPS.pose = tmpPose
            myPath.poses.append(tmpPS)

        #goal state is the end
        goal_state = myPath.poses[-1]
        return myPath, goal_state

def main(args=None):
    rclpy.init(args=args)
    navigator_host = navigatorHost()

    rclpy.spin(navigator_host)

    navigator_host.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        