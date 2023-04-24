import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

     return LaunchDescription([
        Node(
            package='localization',
            executable='localization',
            output='screen'
        ),
        Node(
            package='rplidar_ros2',
            executable='rplidar_scan_publisher',
            name='rplidar_scan_publisher',
            namespace='rplidar_ros',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True
            }]
        ),
        Node(
            package='lidar_scan_acq',
            executable='lidar_scan_acq',
            output='screen',
            parameters=[
                {'saveData': False,
                 'maxRange': 12.0}
            ]
        )
    ])
