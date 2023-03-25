import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        #This is used if you plan to control the robot directly from the
        #vnc server. If not you likely want to start the teleop_twist_keyboard
        #from the controlling node
        #Node(
        #    package='teleop_twist_keyboard',
        #    executable='teleop_twist_keyboard',
        #    output='screen',
        #    prefix='xterm -e'),
        Node(
            package='ssh_driver',
            executable='ssh_driver'),
        Node(
            package='camera_pkg',
            namespace='camera_pkg',
            executable='camera_node',
            name='camera_node'
        ),
        Node(
            package='servo_pkg',
            namespace='servo_pkg',
            executable='servo_node',
            name='servo_node'
        ),
        #Add nodes needed for data collection
        Node(
            package='rplidar_ros2',
            executable='rplidar_scan_publisher',
            name='rplidar_scan_publisher',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }]
        )

            
   ])

