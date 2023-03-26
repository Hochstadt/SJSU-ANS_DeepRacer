#ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from deepracer_interfaces_pkg.msg import CameraMsg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from deepracer_interfaces_pkg.srv import VideoStateSrv
from laser_geometry import LaserProjection

#Python
import os
import math
import pickle
from datetime import datetime

class dataCollector(Node):
        
        lp = LaserProjection()
        #SAVE_RATE seconds has passed before new measurement
        SAVE_RATE = 0.5

        def __init__(self):
            super().__init__('data_collector')
            #qos is needed to read the /scan

            #IF false don't collect data
            self.bCollectData = False

            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
                )
            #Use parameter to get the location to save things
            #Create parameter save_dir, with default of the current 
            #working directory
            cur_dir = os.getcwd()
            self.declare_parameter('save_dir', cur_dir)
            
            self.lidar_subscription = self.create_subscription(
                LaserScan,
                'scan',
                self.lidar_listener_callback,
                qos_profile=qos_profile)
            
            self.camera_subscription = self.create_subscription(
                 CameraMsg,
                 '/camera_pkg/video_mjpeg',
                 self.camera_listener_callback,
                 qos_profile=qos_profile
            )

            #Create a subsriber that keys off the same message the 
            #cameras do
            self.activate_srv = self.create_service(VideoStateSrv,
                'activate_data_collection',
                self.activate_callback)
            
            #Since this is an instance where the save_dir param
            #is set at runtime and that's it we'll access it 
            #in the constructor to setup the save fils
            save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
            c_date = datetime.today()
            date_str = c_date.strftime("%Y_%m_%d_%H_%M_%S")
            self.data_dir = os.path.join(save_dir, date_str)
            os.mkdir(self.data_dir)
            self.get_logger().info('Save directory setup in: %s' % self.data_dir)
            self.cam_time = datetime.now()
            self.lidar_time = self.cam_time
        def activate_callback(self, request, response):
            if request.activate_video == 1:
                self.get_logger().info('Starting data collection')
                self.bCollectData = True
                response.error = 0
            else:
                self.get_logger().info('NOT Starting data collection')
                self.bCollectData = False
                response.error = 0

            return response
            
        def lidar_listener_callback(self, msg):
            #Check if enought ime has apssed
             
            c_time = datetime.now()
            duration = c_time - self.lidar_time            
            #NOTE: LIDAR SAVE RATE IS GOING TO BE 2x the speedo f camera
            if duration.total_seconds > SAVE_RATE/2.0 and self.bCollectData:
                self.get_logger().info('Saving lidar')
                pc2_msg = self.lp.projectLaser(msg)
                tstamp = c_time.strftime("%d_%H_%M_%S_%f")
                fname = tstamp + '_pc2.pickle'
                fname = os.path.join(self.data_dir, fname)
                with open(fname, 'wb') as handle:
                    pickel.dump(pc2_msg, handle)
                #reset time
                self.lidar_time = c_time
                    

        def camera_listener_callback(self, msg):
            #Check if enough time has passed
            c_time = datetime.now()
            duration = c_time - self.cam_time
            if duration.total_seconds() > SAVE_RATE and self.bCollectData:
                self.get_logger().info('Saving Camera IMG')

                #Gen filename
                tstamp  = c_time.strftime("%d_%H_%M_%S_%f")
                fname = tstamp + '_img.pickle'
                fname = os.path.join(self.data_dir, fname)
                with open(fname, 'wb') as handle:
                    pickle.dump(msg.images, handle)
                #Reset time
                self.cam_time = c_time


def main(args=None):
    rclpy.init(args=args)
    
    data_collector = dataCollector()
    print('Spinning Node...')
    rclpy.spin(data_collector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print('Destroying Node')
    data_collector.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
