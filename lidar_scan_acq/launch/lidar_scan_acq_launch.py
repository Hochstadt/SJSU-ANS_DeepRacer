import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

     lidar_scan_acq = Node(
        package='lidar_scan_acq',
        executable='lidar_scan_acq',
        output='screen'
    ) 
    
     return LaunchDescription([
        lidar_scan_acq
    ])
