import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


#Python
import os
import pickle
import numpy as np
from time import sleep

class mapLoader(Node):
    def __init__(self):
        super().__init__('map_loader')
        self.bMapLoaded = False
        map_file = 'na'
        occ_file = 'na'
        goal_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        #Expecting this to be the full path to the map file. If na will search
        #the current directory for one
        self.declare_parameter('map_file', map_file)
        #Only add this if needed....
        #self.declare_parameter('occ_file', occ_file)

        self.declare_parameter('goal_state', goal_state)
        self.map_publisher = self.create_publisher(PointCloud2, 
                                                   '/ans_services/map_pt_msg', 
                                                   10)
        self.map_timer = self.create_timer(1, self.publishMap)

        self.goal_publisher = self.create_publisher(PoseStamped, 
                                                   '/ans_services/goal_state_msg', 
                                                   10)
        self.goal_timer = self.create_timer(1, self.publishGoalState)

        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
                )

        self.pose_subscriber = self.create_subscription(PoseStamped,
                                                        '/localization/pose',
                                                        self.pose_listener, 
                                                        qos_profile=qos_profile)
        

        self.debug_point_cloud = self.create_subscription(PointCloud2,
                                                        '/localization/aligned_lidar_pt_msg',
                                                        self.pt_cloud_listener, 
                                                        qos_profile=qos_profile)
        

        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value
        self.goal_state = self.get_parameter('goal_state').get_parameter_value().double_array_value
    def publishMap(self):
        #Check if this map file exists 
        if self.bMapLoaded == False:       
            if os.path.exists(self.map_file) == False:
                #If not do a searcho f the current directory and take the file
                #with map in the name
                cur_dir = os.curdir
                files = os.listdir(cur_dir)
                for f in files:
                    if 'map' in f and 'pickle' in f:
                        self.get_logger().info('Found map file "%s"' % f)
                        self.map_file = os.path.join(cur_dir, f)
                        break

            self.get_logger().info('Loading file "%s"' % self.map_file)
            if not ('.pickle' in self.map_file and 'map' in self.map_file):
                self.get_logger().error('Could not load file')
                
            #Load the map file
            with open(self.map_file, 'rb') as handle:
                np_pointcloud = pickle.load(handle)
            #Verify in row by column shape
            if np_pointcloud.shape[0] > np_pointcloud.shape[1]:
                np_pointcloud = np_pointcloud.transpose()
            #Need to convert to a poitn cloud shape that's iterable, soa  list
            #where each list element are the three values for the point
            list_pointcloud = []
            
            for i in range(0, np_pointcloud.shape[1]):
                x = np_pointcloud[0, i]
                y = np_pointcloud[1, i]
                if np_pointcloud.shape[0] > 2:
                    z = np_pointcloud[2, i]
                else:
                    z = 0.0
                list_pointcloud.append([x,y,z])

            #Now convert to pointcloud2 using utility
            #Create blank header
            my_header = Header()
            #hopefully this works...
            my_header.stamp = self.get_clock().now().to_msg()
            my_header.frame_id = 'environment_map'
            pc = point_cloud2.create_cloud_xyz32(my_header, list_pointcloud)
            #Now attempt a publish
            self.get_logger().info('Sending Point Cloud To Car')

            


            while rclpy.ok() and self.bMapLoaded == False:        
                self.map_publisher.publish(pc)
                self.bMapLoaded = True
                sleep(0.25)
        
    def publishGoalState(self):

        #Check if this map file exists 
        my_header = Header()
        my_header.stamp = self.get_clock().now().to_msg()
        my_header.frame_id = 'goal_state'
        my_pose = PoseStamped()
        
        my_pose.pose.position.x = self.goal_state[0]
        my_pose.pose.position.y = self.goal_state[1]
        my_pose.pose.position.z = self.goal_state[2]
        my_pose.pose.orientation.x = self.goal_state[3]
        my_pose.pose.orientation.y = self.goal_state[4]
        my_pose.pose.orientation.z = self.goal_state[5]
        my_pose.pose.orientation.w = self.goal_state[6]

        self.goal_publisher.publish(my_pose)

    def pose_listener(self, msg):
        self.get_logger().info('Displaying the estimated pose')
        point = msg.pose.position
        quat = msg.pose.orientation

        self.get_logger().info('Position: <%.4f %.4f %.4f>' % (point.x, point.y, point.z))
        self.get_logger().info('Quaternion: <%.4f %.4f %.4f, %.4f>' % (quat.x, quat.y, quat.z, quat.w))

    def pt_cloud_listener(self, msg):

        self.get_logger().info('Saving Debug Aligned Map')
        point_list = point_cloud2.read_points_list(msg)
        pl_length = len(point_list)
        point_array = np.empty((3, pl_length))
        for i, p in enumerate(point_list):
            point_array[0, i] = p.x
            point_array[1,i] = p.y
            point_array[2,i] = p.z
        with open('debug_map.pickle', 'wb') as handle:
                pickle.dump(point_array, handle)


            

        




def main(args=None):
    rclpy.init(args=args)
    map_loader = mapLoader()


    rclpy.spin(map_loader)

    map_loader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        