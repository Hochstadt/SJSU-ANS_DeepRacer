# Python core
import sys
from functools import partial
from threading import Thread
from time import sleep
from datetime import datetime
import signal
import yaml
import glob
import pathlib

# ROS -- rclpy
import rclpy
from rclpy.node import Node
# ROS -- topics
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from ans_interfaces.srv import DataCollectorState
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseStamped
from deepracer_interfaces_pkg.srv import VideoStateSrv

# PyQt5
# from MainWindow import Ui_MainWindow
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QThread, QTimer, QPoint
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QPen

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
    'WAIT_RESPONSE'  : ['Waiting for response...',  'font: 12pt "Arial"; background-color: yellow;     border: 2px solid black;'],
    'STANDBY'        : ['Standby',                  'font: 12pt "Arial"; background-color: lightgreen; border: 2px solid black;'],
    'DATA_COLLECT'   : ['Data collect in progress', 'font: 11pt "Arial"; background-color: lightblue;  border: 2px solid black;'],
    'NAVIGATION'     : ['Navigation in progress',   'font: 12pt "Arial"; background-color: lightblue;  border: 2px solid black;']
}

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)

        # Additional UI Setup
        self.map_label.setStyleSheet('border: 1px solid black;')
        self.image_label.setScaledContents(True)           # Image label
        self.image_label.setMinimumSize(2, 2)
        self.joystick = Joystick(self.update_joystick_pos) # Joystick
        grid_layout = QGridLayout()
        self.joystick_widget.setLayout(grid_layout)
        grid_layout.addWidget(self.joystick)
        self.goal_pen = QPen()
        self.goal_pen.setWidth(2)
        self.goal_pen.setColor(QColor('red'))
        self.current_pos_pen = QPen()
        self.current_pos_pen.setWidth(3)
        self.current_pos_pen.setColor(QColor('blue'))
        self.path_pen = QPen()
        self.path_pen.setWidth(1)
        self.path_pen.setColor(QColor('blue'))

        # State variables
        self.data_collect_active=False
        self.video_active=False
        self.navigation_active=False
        self.mode='STANDBY'
        self.last_twist_pub=datetime.now()
        self.joystick_x=0.0
        self.joystick_y=0.0
        self.reset_motor=False
        self.occ_yml=None
        self.pcd_map_filename=''
        self.occ_yml_filename=''
        self.map_loaded=False
        self.occ_pixmap=None
        self.goal_pos=None
        self.current_pose=None
        # DEBUG
        self.path=[]#None

        # ROS init
        self.init_ros_node()

        # Button callbacks
        self.config_button.clicked.connect(self.load_config_folder)
        self.data_collect_button.clicked.connect(self.toggle_data_collect)
        self.navigation_button.clicked.connect(self.toggle_navigation_mode)

        # Clicking on the map
        self.map_label.mousePressEvent = self.update_goal

        # Construct CvBridge
        self.bridge = CvBridge()

        # Activate video
        self.send_activate_video()
        self.update_status(self.mode)

    def init_ros_node(self):
        rclpy.init(args=None)

        self.node = Node('ans_gui_node')

        # Filename globs to grab point cloud and occupancy maps
        self.node.declare_parameter('pcd_map_filename', '*final_map*.pickle')
        self.pcd_map_filename = self.node.get_parameter('pcd_map_filename').get_parameter_value().string_value
        self.node.declare_parameter('occ_yml_filename', '*occupancy_map*.yaml')
        self.occ_yml_filename = self.node.get_parameter('occ_yml_filename').get_parameter_value().string_value

        # Subscribers
        self.image_sub = self.node.create_subscription(
            Image,
            'display_mjpeg',
            self.update_image,
            3)
        self.image_count = 0
        self.local_pose_sub = self.node.create_subscription(
            PoseStamped,
            "/localization/pose",
            self.update_current_pose,
            1)
        self.path_sub = self.node.create_subscription(
            Path,
            "/path_planner/path",
            self.update_path,
            1)

        # Publishers
        self.point_cloud_map_pub = self.node.create_publisher(
            PointCloud2,
            "/ans_services/map_pt_msg",
            10)
        self.occupancy_map_pub = self.node.create_publisher(
            OccupancyGrid,
            "/ans_services/occupancy_map_msg",
            10)
        self.goal_state_pub = self.node.create_publisher(
            PoseStamped,
            '/ans_services/goal_state_msg',
            10)
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

    def update_map_display(self):
        if(self.occ_pixmap):
            new_pixmap = QPixmap(self.occ_pixmap)
        else:
            self.node.get_logger().info(f'update_map_display - Occupancy pixmap not loaded')
            return
        
        if self.goal_pos:
            painter = QPainter(new_pixmap)
            painter.setPen(self.goal_pen)
            # Draw red x symbol for the goal
            cross_size=6
            c_x = self.goal_pos.x()
            c_y = self.goal_pos.y()
            top_left  = QPoint(c_x - cross_size / 2, c_y - cross_size / 2)
            bot_right = QPoint(c_x + cross_size / 2, c_y + cross_size / 2)
            top_right = QPoint(c_x + cross_size / 2, c_y - cross_size / 2)
            bot_left  = QPoint(c_x - cross_size / 2, c_y + cross_size / 2)
            painter.drawLine(top_left, bot_right)
            painter.drawLine(top_right, bot_left)
            painter.end()
                
        if self.path:
            painter = QPainter(new_pixmap)
            painter.setPen(self.path_pen)
            # Draw point for each path, with lines between
            prev_point=None
            for path in self.path:
                # Pose
                circle_size=3
                c_x = path.position.x
                c_y = path.position.y
                x_0 = c_x - circle_size / 2.0
                y_0 = c_y - circle_size / 2.0
                painter.drawEllipse(x_0, y_0, circle_size, circle_size)
                if prev_point:
                    painter.drawLine(prev_point, QPoint(c_x, c_y))
                prev_point = QPoint(c_x, c_y)
            painter.end()

        if self.current_pose:
            painter = QPainter(new_pixmap)
            painter.setPen(self.current_pos_pen)
            # Draw blue square for the current position
            square_size=6
            c_x = self.current_pose.pose.position.x
            c_y = self.current_pose.pose.position.y
            x_0 = c_x - square_size / 2
            y_0 = c_y - square_size / 2
            painter.drawRect(x_0, y_0, square_size, square_size)
            painter.end()

        self.map_label.setPixmap(new_pixmap)
        self.map_label.setFixedSize(self.occ_pixmap.width(), self.occ_pixmap.height())
        self.map_label.repaint()

    def update_image(self, msg):
        self.image_count += 1
        if(self.image_count % VIDEO_RATE == 0):
            self.image_count = 0
            # Is this double conversion the best way to display bgr8?
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, 'rgb8')
            image        = QImage(img_msg.data, img_msg.width, img_msg.height, img_msg.step, QImage.Format_RGB888)
            pixel_map    = QPixmap.fromImage(image)
            pixmap_image = QPixmap(pixel_map)
            self.image_label.setPixmap(pixmap_image)

    def update_current_pose(self, msg):
        self.current_pose = msg

    def update_path(self, msg):
        self.path = msg

    def update_goal(self, event):
        # Permit new goal only if a map has been loaded and we're in standby
        if not self.map_loaded and self.mode == 'STANDBY':
            return

        self.goal_pos = event.pos()
        self.node.get_logger().info(f'update_goal - pos={self.goal_pos}')

        # TODO - We may not need to do any transforms here, since we're displaying
        #        the map as-is, so event position == map position?
        new_goal = PoseStamped()
        new_goal.header.frame_id = "odom"
        new_goal.pose.position.x = float(self.goal_pos.x())
        new_goal.pose.position.y = float(self.goal_pos.y())
        new_goal.pose.position.z = 0.0
        new_goal.pose.orientation.x = 0.0
        new_goal.pose.orientation.y = 0.0
        new_goal.pose.orientation.z = 0.0
        new_goal.pose.orientation.w = 0.0

        self.update_map_display()
        self.goal_state_pub.publish(new_goal)

    # Joystick widget constantly calls this as the joystick moves
    def update_joystick_pos(self, x, y, reset=False):
        self.joystick_x = x
        self.joystick_y = y
        self.reset_motor = reset

    def publish_twist_msg(self):
        x = self.joystick_x
        y = self.joystick_y

        # We don't want to keep publishing 0s. Only publish if intentional, when reset is set to True
        if not self.reset_motor and (x >= -0.01 and x <= 0.01) and (y >= -0.01 and y <= 0.01):
            # self.node.get_logger().info(f"publish_twist_msg - TOO LOW TO PUBLISH -- {self.joystick_x},{self.joystick_y}")
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
        full_occ_map_filename=None

        # Directory explorer
        dir_explorer = QFileDialog(self, "Choose directory")
        dir_explorer.setFileMode(QFileDialog.DirectoryOnly)
        dir_explorer.setViewMode(QFileDialog.List)
        dir_explorer.setOption(QFileDialog.ShowDirsOnly)
        dir_explorer.setOption(QFileDialog.DontUseNativeDialog)
        dir_explorer.setOption(QFileDialog.HideNameFilterDetails)
        dir_explorer.exec_()
        directory = dir_explorer.selectedFiles()[0]

        self.node.get_logger().info(f'load_config_folder - Loading in files from {directory}')

        # Get last file that matches the glob parameter for the file
        try:
            full_occ_yml_filename = sorted(glob.glob(f'{directory}/{self.occ_yml_filename}'))[-1]
            self.node.get_logger().info(f'load_config_folder - Loaded {full_occ_yml_filename}')
        except:
            self.node.get_logger().warn(f'load_config_folder - Could not load OCC YAML matching {directory}/{self.occ_yml_filename}')

        try:
            full_pcd_map_filename = sorted(glob.glob(f'{directory}/{self.pcd_map_filename}'))[-1]
            self.node.get_logger().info(f'load_config_folder - Loaded {full_pcd_map_filename}')
        except:
            self.node.get_logger().warn(f'load_config_folder - Could not load PCD matching {directory}/{self.pcd_map_filename}')

        try:
            with open(full_occ_yml_filename, 'r') as yaml_file:
                self.occ_yml = yaml.safe_load(yaml_file)
                full_occ_map_filename = pathlib.Path(directory).joinpath(self.occ_yml['image'])
            self.node.get_logger().info(f'load_config_folder - Loaded {full_occ_map_filename}')
        except:
            self.node.get_logger().warn(f'load_config_folder - Error parsing occupancy YAML file {full_occ_yml_filename}')

        if full_occ_map_filename:
            # TODO - The API that loads in the occupancy grid and PCD map are only supported in C++.
            #        Can we send the filepath to the files? It looks like that functionality may have been
            #        removed?

            # occ_grid = OccupancyGrid()
            # loadMapFromYaml(full_occ_map_filename, occ_grid);
            # occupancy_map_pub.publish(occupancy_grid_msg)

            # pcd_map_msg = PointCloud2()
            # load PCD from full_pcd_map_filename
            # point_cloud_map_pub.publish(pcd_map_msg)
            # 

            self.occ_pixmap = QPixmap(str(full_occ_map_filename.resolve()))
            self.update_map_display()
            self.map_loaded=True

    def toggle_navigation_mode(self):
        self.node.get_logger().info(f'toggle_navigation_mode - Sending request - Navigation mode ({not self.navigation_active})')
        self.node.get_logger().info(f'toggle_navigation_mode - Not implemented though, doing nothing')

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

    def send_activate_video(self, activate_video=1):
        self.node.get_logger().info(f'send_activate_video - Sending request - Video State Activate=({activate_video})')
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
