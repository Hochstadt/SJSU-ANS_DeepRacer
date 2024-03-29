#ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from deepracer_interfaces_pkg.srv import VideoStateSrv
from ans_interfaces.srv import DataCollectorState
from laser_geometry import LaserProjection
from cv_bridge import CvBridge

#Python
import os
import math
import pickle
from datetime import datetime
import cv2 as cv
import threading

class dataCollector(Node):

        lp = LaserProjection()
        #SAVE_RATE seconds has passed before new measurement
        LIDAR_SAVE_RATE = .2
        IMG_SAVE_RATE = 1
        CAMERA_IDX_LIST = [4,3,2,1,0]
        DEFAULT_IMAGE_WIDTH = 160
        DEFAULT_IMAGE_HEIGHT = 120

        def __init__(self):
            super().__init__('data_collector')
            #qos is needed to read the /scan

            #IF false don't collect data
            self.bCollectData = False
            self.bStreamVideo = False

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

            self.video_capture_list = []
            self.video_index_list = []
            self.scanCameraIndex(self.CAMERA_IDX_LIST)
            self.bridge = CvBridge()
            self.videoWorker = threading.Thread()

            self.lidar_subscription = self.create_subscription(
                LaserScan,
                'scan',
                self.lidar_listener_callback,
                qos_profile=qos_profile)

            #Now create publisher to JUST stream
            self.streamer = self.create_publisher(
                 Image,
                 'display_mjpeg',
                 1
            )

            #Create a subsriber that keys off the same message the
            #cameras do
            self.activate_srv = self.create_service(VideoStateSrv,
                'activate_data_collection',
                self.activate_callback)

            self.state_srv = self.create_service(DataCollectorState,
                'data_collector_state',
                self.state_callback)

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

        def start_video(self):
            i = 0
            for cap in self.video_capture_list:
                if not (cap.isOpened()) or not (cap.get(cv.CAP_PROP_FOURCC) == cv.VideoWriter.fourcc(*"MJPG")):
                    self.get_logger().error('Unable to get MJPEG stream: %d' % self.video_index_list[i])
                i = i + 1
            self.videoWorker = threading.Thread(target=self.streamFrames, daemon=True)
            self.videoWorker.start()
            return True

        def activate_callback(self, request, response):
            success = True
            if request.activate_video == 1:
                if self.bStreamVideo == True:
                    self.get_logger().info('Video active requested when video is already streaming, ignoring..')
                    response.error = False
                    return response
                self.get_logger().info('Starting video stream (Data collect command moved to the GUI! Edit me to change)')
                # If you want to tie video + data collect together uncomment below
                # self.bCollectData = True
                self.bStreamVideo = True
                success = success and self.start_video() # bitwise and
            else:
                self.get_logger().info('NOT starting video stream')
                self.bStreamVideo = False
                success = success and True

            response.error = not success
            return response

        def state_callback(self, request, response):
            success = True
            # Data collection also triggers video activation
            if request.collect_data == 1:
                self.get_logger().info('Starting data collection + streaming video')
                self.bCollectData = True
                if self.bStreamVideo == False:
                    self.bStreamVideo = True
                    success = success and self.start_video() # bitwise and
            else:
                self.get_logger().info('NOT starting data collection')
                self.bCollectData = False
                success = success and True

            response.error = not success
            return response

        def lidar_listener_callback(self, msg):
            #Check if enought ime has apssed

            c_time = datetime.now()
            duration = c_time - self.lidar_time
            if duration.total_seconds() > self.LIDAR_SAVE_RATE and self.bCollectData:
                pc2_msg = self.lp.projectLaser(msg)
                tstamp = c_time.strftime("%d_%H_%M_%S_%f")
                fname = tstamp + '_pc2.pickle'
                fname = os.path.join(self.data_dir, fname)
                with open(fname, 'wb') as handle:
                    pickle.dump(pc2_msg, handle)

                #reset time
                self.lidar_time = c_time

            duration = c_time - self.cam_time
            if duration.total_seconds() > self.IMG_SAVE_RATE and self.bCollectData:
                #get frame
                frames = self.produceFrames()
                tstamp = c_time.strftime("%d_%H_%M_%S_%f")
                fname = tstamp + '_img.pickle'
                fname = os.path.join(self.data_dir, fname)
                with open(fname, 'wb') as handle:
                    pickle.dump(frames, handle)
                self.cam_time = c_time

        def produceFrames(self):
            frames = -1
            if self.bCollectData:
                frames = []
                i = 0
                for cap in self.video_capture_list:
                    if cap.isOpened():
                        ret, frame = cap.read()
                        if not ret:
                            self.get_logger().error('Camera index %d did not return frames' % i )
                        else:
                            try:
                                frames.append(frame)
                            except CvBridge.CvBridgeError as e:
                                self.get_logger().error('Cv bridge exception %s' % e)
                                self.bCollectData = False
                    else:
                        self.get_logger().error('Camera index %d not open' % i )
                    i = i + 1
            return frames

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

        def streamFrames(self):
            self.get_logger().info('Enter streamFrames')
            while self.bStreamVideo:
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
                            self.bCollectData = False

                        msg.header.frame_id = 'base_scan'
                        #Now attempt to actually publish
                        self.streamer.publish(msg)
                else:
                    self.get_logger().error('Camera at index 0 not opened')

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
