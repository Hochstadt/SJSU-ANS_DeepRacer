#ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

#Python
import os
import math
import pickle
from datetime import datetime

#Left off - need to run the whole data collection scheme. Add rviz node to the launch 
#script of the controller. Then get back to paradigm of runnign one thing on
#car and one thing on the host. This interface is written, but not built, 
# or tested, but it owuld be added to the launcher & setup as well
class rvizInterface(Node):
    lp = LaserProjection()
    PUBLISH_EVERY = 20
    def __init__(self):
            super().__init__('rviz_interface')
            #NOTE: qos is needed to read the /scan
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
                )

            
            self.lidar_subscription = self.create_subscription(
                LaserScan,
                'scan',
                self.lidar_listener_callback,
                qos_profile=qos_profile)
            
            self.publisher_ = self.create_publisher(
                 PointCloud2,
                 '/rviz_interface/lidar_pt_msg',
                 10
            )
            self.pub_count = 0

    def lidar_listener_callback(self, msg):
        
        pc2_msg = self.lp.projectLaser(msg)
        #call publisher to publish       
        if self.pub_count == self.PUBLISH_EVERY:
            self.publisher_.publish(pc2_msg)
            self.pub_count = 0
        else:
            self.pub_count+=1

def main(args=None):
    rclpy.init(args=args)
    
    rviz_inf = rvizInterface()
    print('Spinning Node...')
    rclpy.spin(rviz_inf)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rviz_inf.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()