# Python core
import sys
from functools import partial
from threading import Thread
from time import sleep
import signal

# ROS -- rclpy
import rclpy
from rclpy.node import Node
# ROS -- topics
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from ans_interfaces.srv import DataCollectorState
# ROS -- ?
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from deepracer_interfaces_pkg.srv import VideoStateSrv

# PyQt5
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QThread, QTimer
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QImage, QPixmap, QFont

import numpy as np

# Signals to terminate
def _signal_handler(*args):
    _global_running=False
    signal.SIG_DFL
    QApplication.quit()

signal.signal(signal.SIGINT, _signal_handler)
_global_running=True

class AnsGuiNode(Node):
    def __init__(self, blocking_init=True):
        super().__init__('ans_gui_node')
        self.node_init_complete=False
        self.video_active=False
        self.data_collect_active=False

        self.data_collector_state_client = self.create_client(DataCollectorState, 'data_collector_state')
        self.video_state_client = self.create_client(VideoStateSrv, 'activate_data_collection')

        self.init_thread = Thread(target=self.init_clients, daemon=not blocking_init)
        self.init_thread.start()
        if(blocking_init):
            self.init_thread.join()

    def init_clients(self):
        while not self.video_state_client.wait_for_service(timeout_sec=2.0) and _global_running:
            self.get_logger().info('video_state_service not available, waiting again...')
        while not self.data_collector_state_client.wait_for_service(timeout_sec=2.0) and _global_running:
            self.get_logger().info('data_collector_state_service not available, waiting again...')
        if(_global_running):
            self.get_logger().info('Services up! Requesting...')
            self.data_collector_request = DataCollectorState.Request()
            self.video_state_request = VideoStateSrv.Request()
            self.node_init_complete = True

    def send_data_collector_request(self, collect_data):
        if not self.node_init_complete:
            self.get_logger().warn(f'send_data_collector_request - Init not completed')
            return 1

        if self.data_collector_request.collect_data == collect_data:
            self.get_logger().warn(f'send_data_collector_request - Data collect already set to {collect_data}?')
            return 1

        self.get_logger().info(f'send_data_collector_request - Sending data collector requesting {collect_data}...')
        self.data_collector_request.collect_data = collect_data
        self.future = self.data_collector_state_client.call_async(self.data_collector_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().error

    def send_video_state_request(self, activate_video):
        if not self.node_init_complete:
            self.get_logger().warn(f'send_video_state_request - Init not completed')
            return 1

        if self.video_state_request.activate_video == activate_video:
            self.get_logger().warn(f'send_video_state_request - Video state already set to {activate_video}?')
            return 1

        self.get_logger().info(f'send_video_state_request - video state requesting {activate_video}')
        self.video_state_request.activate_video = activate_video
        self.future = self.video_state_client.call_async(self.video_state_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().error

# class MotorLogic:
#     """MotorLogic's controller class."""

#     def __init__(self, arrows_window, log_window):
#         self._arrows_window = arrows_window
#         self._connectSignalsAndSlots()

#     def _direction(self, keySymbol):
#         print(f'Pressed {keySymbol}')
#         # self._arrows_window.setDisplayText(expression)

#     def _connectSignalsAndSlots(self):
#         for keySymbol, button in self._arrows_window.buttonMap.items():
#             button.clicked.connect(partial(self._direction, keySymbol))

# WINDOW_SIZE = 235
# DISPLAY_HEIGHT = 40
# BUTTON_SIZE = 40

# class MotorControlWindow(QMainWindow):
#     """MotorLogic's main window (GUI or view)."""

#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Motor Control")
#         self.setFixedSize(WINDOW_SIZE, WINDOW_SIZE)
#         self.generalLayout = QVBoxLayout()
#         centralWidget = QWidget(self)
#         centralWidget.setLayout(self.generalLayout)
#         self.setCentralWidget(centralWidget)
#         self._createButtons()

#     def _createButtons(self):
#         self.buttonMap = {}
#         buttonsLayout = QGridLayout()
#         arrow_buttons = [
#             ["FL", "FF", "FR"],
#             ["EL", "SS", "ER"],
#             ["BL", "BB", "BR"],
#         ]
#         for row, buttons in enumerate(arrow_buttons):
#             for col, key in enumerate(buttons):
#                 # self.buttonMap[key] = QPushButton(key)
#                 self.buttonMap[key] = QPushButton()
#                 pixmapi = getattr(QStyle, 'SP_ArrowUp')
#                 icon = self.style().standardIcon(pixmapi)
#                 self.buttonMap[key].setIcon(icon)
#                 self.buttonMap[key].setFixedSize(BUTTON_SIZE, BUTTON_SIZE)
#                 buttonsLayout.addWidget(self.buttonMap[key], row, col)
#         self.generalLayout.addLayout(buttonsLayout)

# class VideoThread(QThread):
#     change_pixmap_signal = pyqtSignal(np.ndarray)

#     def __init__(self, video_data_cap):
#         super().__init__()
#         self._run_flag = True
#         self.video_data_cap = video_data_cap

#     def run(self):
#         # capture from web cam
#         # cap = cv2.VideoCapture(0)
#         print("VideoThread started")
#         while(self.video_data_cap.dumb_variable[0] == False):
#             # Busy wait :(
#             sleep(2)

#         print("Starting video read")
#         cap = self.video_data_cap.video_capture_list[0]
#         while self._run_flag:
#             ret, cv_img = cap.read()
#             if ret:
#                 self.change_pixmap_signal.emit(cv_img)
#         # shut down capture system
#         cap.release()

#     def stop(self):
#         """Sets run flag to False and waits for thread to finish"""
#         self._run_flag = False
#         self.wait()

class AnsGui(QWidget):
    def __init__(self, ans_gui_node):
        super().__init__()

        # Assign node + use node logger
        self.node = ans_gui_node
        self.logger = self.node.get_logger()

        self.data_collect_active=False
        self.init_complete=False

        ###### /SETUP GUI COMPONENTS
        self.setWindowTitle("Controller")
        self.disply_width = 640
        self.display_height = 480
        # self.setFixedSize(self.disply_width*2, self.display_height*1.5)
        # create the label that holds the image
        self.image_label = QLabel(self)
        # self.image_label.setScaledContents(True) # maybe
        self.image_label.resize(self.disply_width, self.display_height) # maybe use setFixedSize
        # create a text label
        self.title_label = QLabel('ANS DeepRacer')
        self.title_label.setScaledContents(True)
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("border: 7px solid black;")
        self.title_label.setFont(QFont('Times', 35))

        # Create status label
        self.status_label = QLabel("Init...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("background-color: yellow; border: 1px solid black;")
        self.status_label.setFont(QFont('Arial', 10))

        self.data_capture_b = QPushButton('Start data collect mode')
        self.red_b  = QPushButton('set red LED')
        self.blue_b = QPushButton('set blue LED')

        # create a vertical box layout and add the two labels
        hbox = QHBoxLayout()
        hbox.addWidget(self.image_label)
        hbox.addWidget(self.status_label)
        hbox.addWidget(self.data_capture_b)
        hbox.addWidget(self.red_b)
        hbox.addWidget(self.blue_b)

        # create a vertical box layout and add the two labels
        vbox = QVBoxLayout()
        vbox.addStretch()
        vbox.addWidget(self.title_label)
        vbox.addLayout(hbox)
        vbox.addStretch()

        # set the vbox layout as the widgets layout
        self.setLayout(vbox)

        self.setGeometry(300, 300, 350, 300)
        # create the video capture thread
        # self.thread = VideoThread(video_data)
        # connect its signal to the update_image slot
        # self.thread.change_pixmap_signal.connect(self.update_image)
        # start the thread
        # self.thread.start()

        # self.video_data = [video_data]

        # Setup GUI callbacks
        self.red_b.clicked.connect(self.red_button)
        self.blue_b.clicked.connect(self.blue_button)
        self.data_capture_b.clicked.connect(self.toggle_data_capture)
        ###### /SETUP GUI COMPONENTS

        self.update_status('init')

        # Image Subscriber
        self.image_subscription = self.node.create_subscription(
            Image,
            'display_mjpeg',
            self.update_image,
            10)
        self.image_subscription  # prevent unused variable
        self.image_counter = 0 # delete me

        # Wait until node is finished initializing
        self.init_thread = Thread(target=self.update_status_check, daemon=True)
        self.init_thread.start()

    def update_status_check(self):
        while(_global_running and not self.node.node_init_complete):
            # busy wait :(
            sleep(2)
        if(_global_running):
            self.update_status('init_complete')
            self.node.send_video_state_request(1)

    def update_image(self, msg):
        self.image_counter = self.image_counter + 1
        # if self.image_counter % 1 == 0:
        self.logger.info(f'display_mjpeg callback # {self.image_counter}')
        self.logger.info(f"  image_counter: {self.image_counter}\n"
                            f"  height:        {msg.height}\n"
                            f"  width:         {msg.width}\n"
                            f"  encoding:      {msg.encoding}\n"
                            f"  is_bigendian:  {msg.is_bigendian}\n"
                            f"  step:          {msg.step}")

    def red_button(self):
        # self.logger.info(f'AnsGui::red_button no action')
        self.logger.info(f'AnsGui::red_button sending video request 1')
        self.node.send_video_state_request(1)
        # msg = String()
        # msg.data = "red"
        # self.video_data[0].publisher_.publish(msg)

    def blue_button(self):
        self.logger.info(f'AnsGui::blue_button no action')
        # msg = String()
        # msg.data = "blue"
        # self.video_data[0].publisher_.publish(msg)

    def update_status(self, mode=''):
        label='Standby'
        style='background-color: lightgreen'

        # if not all([node.data_collector_client_ready, node.video_state_client_ready]):
        #     self.init_complete=False
        #     label='Waiting for services...'
        #     style='background-color: yellow; border: 1px solid black;'
        if mode == 'init':
            self.init_complete=False
            label='Waiting for initialization to complete...'
            style='background-color: yellow; border: 1px solid black;'
        elif mode == 'init_complete':
            self.init_complete=True
        elif mode != '':
            label=mode
            style='background-color: darkgreen; border: 2px solid black;'

        self.logger.warn(f'update_status - mode = {mode}, new label = {label}')
        self.status_label.setStyleSheet(style)
        self.status_label.setText(label)

    def toggle_data_capture(self):
        if not self.init_complete:
            self.logger.warn(f'toggle_data_capture - Init not completed')
            return

        self.logger.info(f'toggle_data_capture - Sending data collector request to {not self.data_collect_active}')
        error = self.node.send_data_collector_request(not self.data_collect_active)
        if error:
            self.logger.warning(f'AnsGui::toggle_data_capture - Failed to toggle data_collect_active')
        else:
            self.data_collect_active = not self.data_collect_active
            self.logger.info(f'AnsGui::toggle_data_capture - Toggled data_collect_active from {not self.data_collect_active} to {self.data_collect_active}')

            if self.data_collect_active:
                text_suffix='Stop'
                status='Collecting data...'
            else:
                text_suffix='Start'
                status=''
            self.data_capture_b.setText(f'{text_suffix} data collect mode')
            self.update_status(status)

    def closeEvent(self, event):
        self.logger.info(f'AnsGui::closeEvent - Exiting')
        self.running = False
        # self.thread.stop()
        if self.data_collect_active:
            error = self.node.send_data_collector_request(0)
            if error:
                self.logger.warning(f'AnsGui::closeEvent - Data collect stop request failed')

    # @pyqtSlot(np.ndarray)
    # def update_image(self, cv_img):
    #     """Updates the image_label with a new opencv image"""
    #     qt_img = self.convert_cv_qt(cv_img)
    #     self.image_label.setPixmap(qt_img)

    # def convert_cv_qt(self, cv_img):
    #     """Convert from an opencv image to QPixmap"""
    #     rgb_image = cv.cvtColor(cv_img, cv.COLOR_BGR2RGB)
    #     h, w, ch = rgb_image.shape
    #     bytes_per_line = ch * w
    #     convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
    #     p = convert_to_Qt_format.scaled(self.disply_width, self.display_height, Qt.KeepAspectRatio)
    #     return QPixmap.fromImage(p)

def main(args=None):
    print("Init...")

    # Python can't handle signals while Qt event loop is executing
    # Create timer to bring control to main thread in the event of a signal
    rclpy.init(args=args)

    ans_gui_node = AnsGuiNode(blocking_init=False)

    app = QApplication([])
    # app.setStyle("Fusion") # or 'Windows'

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(ans_gui_node)
    # executor.add_node(node2)
    # Spin in a separate thread
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = ans_gui_node.create_rate(1)

    # window.show()
    print("Starting app...")
    ans_gui = AnsGui(ans_gui_node)
    ans_gui.show()

    signal_timer = QTimer()
    signal_timer.start(500)
    signal_timer.timeout.connect(lambda: None)

    app.exec_()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print('Destroying Node')
    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()
