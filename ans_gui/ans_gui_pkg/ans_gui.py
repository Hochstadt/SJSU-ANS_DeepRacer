# Python core
import sys
from functools import partial
from threading import Thread
from time import sleep
from datetime import datetime
import signal

# ROS -- rclpy
import rclpy
from rclpy.node import Node
# ROS -- topics
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from ans_interfaces.srv import DataCollectorState
from geometry_msgs.msg import Twist
# ROS -- ?
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from deepracer_interfaces_pkg.srv import VideoStateSrv

# PyQt5
# from MainWindow import Ui_MainWindow
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QThread, QTimer
from PyQt5.QtGui import QImage, QPixmap, QFont

# Developed packages
from ans_gui_pkg.MainWindow import Ui_MainWindow
from ans_gui_pkg.joystick import Joystick

from cv_bridge import CvBridge

# Signals to terminate
def _signal_handler(*args):
    _global_running=False
    signal.SIG_DFL
    QApplication.quit()

signal.signal(signal.SIGINT, _signal_handler)
_global_running=True

VIDEO_RATE=2 # Display video every n frames. If I display every frame the GUI crashes...
TWIST_RATE=1 # Send a twist message at most this many times per second


STATUS_LABEL_STYLE = {
    'INITIALIZATION' : ['Initializing...',          'font: 12pt "Arial"; background-color: yellow;     border: 2px solid black;'],
    'STANDBY'        : ['Standby',                  'font: 12pt "Arial"; background-color: lightgreen; border: 2px solid black;'],
    'DATA_COLLECT'   : ['Data collect in progress', 'font: 11pt "Arial"; background-color: lightblue;  border: 2px solid black;'],
    'NAVIGATION'     : ['Navigation in progress',   'font: 12pt "Arial"; background-color: lightblue;  border: 2px solid black;']
}

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)

        # Additional UI Setup
        self.image_label.setScaledContents(True)           # Image label
        self.image_label.setMinimumSize(2, 2)
        self.joystick = Joystick(self.update_joystick_pos) # Joystick
        grid_layout = QGridLayout()
        self.joystick_widget.setLayout(grid_layout)
        grid_layout.addWidget(self.joystick)

        # ROS init
        self.init_ros_node()

        # State variables
        self.data_collect_active=False
        self.video_active=False
        self.navigation_active=False
        self.mode='STANDBY'
        self.last_twist_pub=datetime.now()
        self.joystick_x = 0.0
        self.joystick_y = 0.0
        self.reset_motor=False

        # Button callbacks
        self.config_button.clicked.connect(self.load_config_folder)
        self.data_collect_button.clicked.connect(self.toggle_data_collect)
        self.navigation_button.clicked.connect(self.toggle_navigation_mode)

        # Construct CvBridge
        self.bridge = CvBridge()

        # Activate video
        self.send_activate_video()
        self.update_status(self.mode)

    def init_ros_node(self):
        rclpy.init(args=None)

        self.node = Node('ans_gui_node')

        # Subscribers
        self.image_sub = self.node.create_subscription(
            Image,
            'display_mjpeg',
            self.update_image,
            10)
        self.image_count = 0

        # Publishers
        self.twist_pub = self.node.create_publisher(
            Twist,
            'cmd_vel',
            1) # We don't really want to queue up servo commands
        timer_period = 1.0 / TWIST_RATE
        self.twist_timer = self.node.create_timer(timer_period, self.publish_twist_msg)

        # Clients
        self.video_state_client = self.node.create_client(VideoStateSrv, 'activate_data_collection')
        while not self.video_state_client.wait_for_service(timeout_sec=2.0) and _global_running:
            self.node.get_logger().info('video_state_service not available, waiting again...')
        self.video_state_request = VideoStateSrv.Request()

        self.data_collector_state_client = self.node.create_client(DataCollectorState, 'data_collector_state')
        while not self.data_collector_state_client.wait_for_service(timeout_sec=2.0) and _global_running:
            self.node.get_logger().info('data_collector_state_service not available, waiting again...')
        self.data_collector_request = DataCollectorState.Request()

        # Spin in a separate thread
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def update_image(self, msg):
        self.image_count += 1
        if(self.image_count % VIDEO_RATE == 0):
            self.image_count = 0
            # TODO - Is this double conversion the best way to display bgr8?
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, 'rgb8')
            image        = QImage(img_msg.data, img_msg.width, img_msg.height, img_msg.step, QImage.Format_RGB888)
            # image        = QImage(msg.data, msg.width, msg.height, msg.step, QImage.Format_BGR888) This is only in PyQt6 >:(
            pixel_map    = QPixmap.fromImage(image)
            pixmap_image = QPixmap(pixel_map)
            self.image_label.setPixmap(pixmap_image)

    # Joystick widget constantly calls this as the joystick moves
    def update_joystick_pos(self, x, y, reset=False):
        self.joystick_x = x
        self.joystick_y = y
        self.reset_motor = reset

    def publish_twist_msg(self):
        x = self.joystick_x
        y = self.joystick_y

        # We don't want to keep publishing 0s. Only publish if intentional, when reset is set to True
        if not self.reset_motor and (x >= 0.0 and x <= 0.05) and (y >= 0.0 and y <= 0.05):
            self.node.get_logger().info(f"publish_twist_msg - TOO LOW TO PUBLISH -- {self.joystick_x},{self.joystick_y}")
            return

        if self.reset_motor:
            self.reset_motor = False

        twist_msg = Twist()

        # Experiemntal scale factors
        speed = self.speed_slider.value() / 30.0 # [0 - 99]
        x = x / 50.0                             # [-50, 50]
        y = y / 50.0                             # [-50, 50]

        # Twist x and Joystick y are forward, Twist z and Joystick x are right
        twist_msg.linear.x  = y * speed
        twist_msg.linear.z  = x * speed

        twist_msg.linear.y  = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0

        # Twist angular z is yaw
        twist_msg.angular.z = - x * speed

        self.node.get_logger().info(f"publish_twist_msg - PUBLISHING -- {self.joystick_x},{self.joystick_y},{speed}"
                                    f"-> fwd={twist_msg.linear.x}, lateral={twist_msg.linear.z}, turn={twist_msg.angular.z}")
        self.twist_pub.publish(twist_msg)

    def load_config_folder(self):
        dir_explorer = QFileDialog(self, "Choose directory")
        dir_explorer.setFileMode(QFileDialog.DirectoryOnly)
        dir_explorer.setViewMode(QFileDialog.List)
        dir_explorer.setOption(QFileDialog.ShowDirsOnly)
        dir_explorer.setOption(QFileDialog.DontUseNativeDialog)
        dir_explorer.setOption(QFileDialog.HideNameFilterDetails)
        dir_explorer.exec_()
        directory = dir_explorer.selectedFiles()[0]

        self.node.get_logger().info(f'load_config_folder - Loading in files from {directory})')
        # We can impose the restriction that all input files should be available in the directory (such as point cloud, occupancy map)
        # with open(f'{directory}/point_cloud.xml', point_cloud):
        #     self.point_cloud = parse_point_cloud(point_cloud)
        # with open(f'{directory}/point_cloud.xml', occ_map):
        #     self.occupancy_map = parse_occupancy_map(occ_map)
        # self.send....

    def toggle_data_collect(self):
        self.node.get_logger().info(f'toggle_data_collect - Sending request - Data Collector Activate ({not self.data_collect_active})')
        self.data_collector_request.collect_data = not self.data_collect_active
        self.data_collector_state_client.call_async(self.data_collector_request)
        self.data_collect_active = not self.data_collect_active

        if(self.data_collect_active == True):
            self.disable_controls_except('DATA_COLLECT')
            self.update_status('DATA_COLLECT')
            text_suffix='Stop'
        else:
            self.enable_all_controls()
            self.update_status('STANDBY')
            text_suffix='Start'

        self.data_collect_button.setText(f'{text_suffix} Data Collect Mode')

    def toggle_navigation_mode(self):
        self.node.get_logger().info(f'toggle_navigation_mode - Sending request - Navigation mode ({not self.navigation_active})')
        self.node.get_logger().info(f'toggle_navigation_mode - Not implemented though, doing nothing)')

        self.navigation_active = not self.navigation_active

        # Here is where I would do some good ol' navigating

        if(self.navigation_active == True):
            self.disable_controls_except('NAVIGATION')
            self.update_status('NAVIGATION')
            text_suffix='Stop'
        else:
            self.enable_all_controls()
            self.update_status('STANDBY')
            text_suffix='Start'

        self.navigation_button.setText(f'{text_suffix} Navigation')

    def send_activate_video(self, activate_video=1):
        self.node.get_logger().info(f'send_activate_video - Sending request - Video State Activate ({activate_video})')
        self.video_state_request.activate_video = activate_video
        self.video_state_client.call_async(self.video_state_request)

    def disable_controls_except(self, exception):
        self.config_button.setDisabled(True)
        self.data_collect_button.setDisabled(True)
        self.navigation_button.setDisabled(True)

        if exception == 'CONFIG':
            self.config_button.setDisabled(False)
        elif exception == 'DATA_COLLECT':
            self.data_collect_button.setDisabled(False)
        elif exception == 'NAVIGATION':
            self.navigation_button.setDisabled(False)

    def enable_all_controls(self):
        self.config_button.setDisabled(False)
        self.data_collect_button.setDisabled(False)
        self.navigation_button.setDisabled(False)

    def update_status(self, mode):
        self.mode = mode
        self.status_label.setText(STATUS_LABEL_STYLE[mode][0])
        self.status_label.setStyleSheet(STATUS_LABEL_STYLE[mode][1])

def main(args=None):
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()
    app.exec()

if __name__ == '__main__':
    main()
