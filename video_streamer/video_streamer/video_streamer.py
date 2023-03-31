#Purpose: to just stream downsampled images to conserve bandwidth
#this seperates the data saving of full sized images and the data
#streaming for driving purposes. Also changes the camera controller
#from C++ to Python modules


#ROS
import rclpy
from rclpy.node import Node
from deepracer_interfaces_pkg.msg import CameraMsg
from deepracer_interfaces_pkg.srv import VideoStateSrv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


#Python
import cv2 as cv
import threading

class videoStreamer(Node):
    CAMERA_IDX_LIST = [4,3,2,1]
    DEFAULT_IMAGE_WIDTH = 160
    DEFAULT_IMAGE_HEIGHT = 120    
    
    def __init__(self):
        super().__init__('video_streamer')
        self.video_capture_list = []
        self.video_index_list = []
        self.bProduceFrames = False
        self.scanCameraIndex(self.CAMERA_IDX_LIST)
        self.bridge = CvBridge()
        self.videoWorker = -1
        #Now create publisher to JUST stream
        self.publisher_ = self.create_publisher(
                 Image,
                 'display_mjpeg',
                 1
            )
        #Create a service to activate the data streaming
        self.activate_srv = self.create_service(VideoStateSrv,
            'media_state',
            self.videoProducerStateHdl)


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
                    self.video_capture_list[-1].set(cv.CAP_PROP_FORCC, cv.VideoWriter.forcc(*"MJPG"))
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

    def produceFrames(self):
        while self.bProduceFrames:
            #Don't need stereo imagery... just the one
            mono_cap = self.video_capture_list[0]
            msg = CameraMsg()
            if mono_cap.isOpened():
                ret, frame = mono_cap.read()
                if not ret:
                    self.get_logger().error('Camera index 0 did not return frames')
                else:
                    frame = cv.resize(frame, self.DEFAULT_IMAGE_WIDTH, self.DEFAULT_IMAGE_HEIGHT)
                    try:
                        msg.images.append(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                    except CvBridge.CvBridgeError as e:
                        self.get_logger().error('Cv bridge exception %s' % e)
                        self.bProduceFrames = False
                    #Now attempt to actually publish                    
                    self.publisher_.publish(msg)    
            else:
                self.get_logger().error('Camera at index 0 not opened')

    def videoProducerStateHdl(self,request, response):
        self.bProduceFrames = False
        
        if not (self.videoWorker == -1) or self.videoWorker.is_alive():
            #Waits for the thread to finish so we can continue to use
            self.videoWorker.join()

        #Check we are collecting from mjpg channel
        i = 0
        for cap in self.video_capture_list:
            if not (cap.isOpened()) or not (cap.get(cv.CAP_PROP_FORCC) == cv.VideoWriter.forcc(*"MJPG")):
                self.get_logger().error('Unable to get MJPEG stream: %d' % self.video_index_list[i])
            i = i + 1
        if request.activate_video == 1:
            self.bProduceFrame = True
            self.videoWorker = threading.Thread(target=self.produceFrames, daemon=True)
        response.error = 0


def main(args=None):
    rclpy.init(args=args)
    video_streamer = videoStreamer()
    print('Spinning Node')
    rclpy.spin(video_streamer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()