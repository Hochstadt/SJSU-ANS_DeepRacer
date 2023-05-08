import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    occ_map_name = DeclareLaunchArgument(
        "occ_map_name", default_value=TextSubstitution(text="occupancy.yaml")
    )


    ans_services= Node(
       package='ans_server',
       executable='cfg_server',
       parameters=[{
                'occ_map': LaunchConfiguration('occ_map_name'),
            }]
    )

    return LaunchDescription([
       occ_map_name,
       ans_services
    ])
