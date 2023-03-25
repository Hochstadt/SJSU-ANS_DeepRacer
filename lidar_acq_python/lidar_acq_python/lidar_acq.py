import rclpy
from rclpy.node import Node
#from deepracer_interfaces_pkg.msg import EvoSensorMsg
import os
import math
import pickle
from datetime import datetime

#Ros specific things
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from deepracer_interfaces_pkg.msg import CameraMsg
from laser_geometry import LaserProjection

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

class lidarAcq(Node):
        
        lp = LaserProjection()
        def __init__(self):
            super().__init__('lidar_acq')
            #NOTE: qos is needed to read the /scan
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
                )
            
            cur_dir = os.getcwd()
            c_date = datetime.today()
            date_str  = c_date.strftime("%Y_%m_%d_%H_%M_%S")
            self.data_dir = os.path.join(cur_dir, date_str)
            os.mkdir(self.data_dir)
            
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
            #self.subscription  # prevent unused variable warning

            self.publisher_ = self.create_publisher(
                 PointCloud2,
                 '/lidar_scan_acq/lidar_pt_msg',
                 10
            )
            self.pub_count = 0
            #self.timer = self.create_timer(.5, self.timer_callback)

        def lidar_listener_callback(self, msg):
            
            self.get_logger().info('LIDAR msg received')
            pc2_msg = self.lp.projectLaser(msg)
            #call publisher to publish       
            if self.pub_count == 20:
                self.publisher_.publish(pc2_msg)
                self.pub_count = 0
            else:
                 self.pub_count+=1
                    

        def camera_listener_callback(self, msg):
             self.get_logger().info('Camera img received')

             #Gen filename
             tstamp = datetime.now()
             tstamp  = tstamp.strftime("%H_%M_%S_%f")
             fname = tstamp + '.pickle'
             fname = os.path.join(self.data_dir, fname)
             with open(fname, 'wb') as handle:
                  pickle.dump(msg.images, handle)

             

#        def timer_callback(self):
#            #lidar_scan_msg = PointCloud2()
#            msg = String()
#            msg.data = 'test12'
#            self.publisher_.publish(msg)
#            self.get_logger().info('Publishing')


            

def main(args=None):
    rclpy.init(args=args)
    
    lidar_acq = lidarAcq()
    print('Spinning Node...')
    rclpy.spin(lidar_acq)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print('Destrotying Node')
    lidar_acq.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()