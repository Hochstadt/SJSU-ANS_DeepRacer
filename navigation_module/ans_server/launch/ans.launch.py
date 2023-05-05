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
        "occ_map_name", default_value=TextSubstitution(text="occ_map.yaml")
    )

    nav_map_name = DeclareLaunchArgument(
        "nav_map_name", default_value=TextSubstitution(text='nav_map.pcd')
    ) 

    goal_state = DeclareLaunchArgument(
        "goal_state", default_value=TextSubstitution(text='[0.44, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]')
    )

    ans_services= Node(
       package='ans_server',
       executable='cfg_server',
       parameters=[{
                'occ_map': LaunchConfiguration('occ_map_name'),
                'nav_map': LaunchConfiguration('nav_map_name'), 
                'goal_state': LaunchConfiguration('goal_state')
            }]
    )

    return LaunchDescription([
       occ_map_name,
       nav_map_name,
       goal_state,
       ans_services
    ])
