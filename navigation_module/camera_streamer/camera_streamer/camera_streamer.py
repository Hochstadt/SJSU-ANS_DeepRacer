#ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from deepracer_interfaces_pkg.srv import VideoStateSrv
from cv_bridge import CvBridge

#Python
import os
import math
import pickle
from datetime import datetime
import cv2 as cv
import threading

class cameraStreamer(Node):
        

        CAMERA_IDX_LIST = [4,3,2,1,0]
        DEFAULT_IMAGE_WIDTH = 160
        DEFAULT_IMAGE_HEIGHT = 120    

        def __init__(self):
            super().__init__('camera_streamer')

            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
                )
            
            self.video_capture_list = []
            self.video_index_list = []
            
            self.bridge = CvBridge()
            self.videoWorker = threading.Thread()
            
            #Now create publisher to JUST stream
            self.streamer = self.create_publisher(
                 Image,
                 'display_mjpeg',
                 1
            )
            
            self.scanCameraIndex(self.CAMERA_IDX_LIST)



        def scanCameraIndex(self, camera_idx_list):
            for camera_id in camera_idx_list:
                self.get_logger().info('Scanning Camera Index %d' % camera_id)
                tmp_cap = cv.VideoCapture(camera_id, cv.CAP_V4L)

                if not tmp_cap.isOpened():
                    self.get_logger().info('Could not open camera index %d' % camera_id)
                else:
                    ret, frame = tmp_cap.read()
                    if not ret:
                        self.get_logger().info('Could not read frames from camera index %d' % camera_id)
                    else:
                        #add to valid capture list
                        self.video_capture_list.append(tmp_cap)
                        self.video_capture_list[-1].set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc(*"MJPG"))
                        self.video_index_list.append(camera_id)
            #Do some general display
            list_len = len(self.video_index_list)
            if list_len == 0:
                self.get_logger().info('No Camera Detected')
            elif list_len == 1:
                self.get_logger().info('Single camera detected at index %d' % self.video_index_list[0])
            elif list_len == 2:
                self.get_logger().info('Stereo cameras detected at indexes: %d, %d' % (self.video_index_list[0], self.video_index_list[1]))
            else:
                self.get_logger().error('Error while detecting cameras')

            #Now start the video 
            #Check we are collecting from mjpg
            i = 0
            for cap in self.video_capture_list:
                if not (cap.isOpened()) or not (cap.get(cv.CAP_PROP_FOURCC) == cv.VideoWriter.fourcc(*"MJPG")):
                    self.get_logger().error('Unable to get MJPEG stream: %d' % self.video_index_list[i])
                i = i + 1
            self.videoWorker = threading.Thread(target=self.streamFrames, daemon=True)
            self.videoWorker.start()

        def streamFrames(self):
            while True:
                #Don't need stereo imagery... just the one
                mono_cap = self.video_capture_list[0]
                if mono_cap.isOpened():
                    ret, frame = mono_cap.read()
                    if not ret:
                        self.get_logger().error('Camera index 0 did not return frames')
                    else:
                        frame = cv.resize(frame, (self.DEFAULT_IMAGE_WIDTH, self.DEFAULT_IMAGE_HEIGHT))
                        try:
                            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        except CvBridge.CvBridgeError as e:
                            self.get_logger().error('Cv bridge exception %s' % e)
                        #Now attempt to actually publish                    
                        self.streamer.publish(msg)    
                else:
                    self.get_logger().error('Camera at index 0 not opened')

def main(args=None):
    rclpy.init(args=args)
    
    camera_streamer= cameraStreamer()
    print('Spinning Node...')
    rclpy.spin(camera_streamer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print('Destroying Node')
    data_collector.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
