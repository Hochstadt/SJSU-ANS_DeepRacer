import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

     localization = Node(
        package='localization',
        executable='localization',
        output='screen'
    ) 
    
     return LaunchDescription([
        localization
    ])
