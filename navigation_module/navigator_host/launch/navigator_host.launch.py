import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration


import os
#Assumes working from the main SJSU repo
#Would use parameters using DeclareLaunchArgument, but 
#cannot combine data folder with output of LaunchConfiguraiton
#so just deifning the default path names here (these are the default
# file names output from map builder though)
MAP_FILE_PCD = 'map_file.pcd'
OCC_FILE_YAML = 'occupancy.yaml'


algorithm_data_folder = 'algorithm_data'
rviz_config_file_name = os.path.join('rviz_configs', 'project_presentation.rviz')
repo_locs = ['', '..', os.path.join('..', '..'), os.path.join('..','..','..')]
for i in repo_locs:
    if os.path.exists(os.path.join(i, rviz_config_file_name)):
        rviz_config = os.path.join(i, rviz_config_file_name)
        break

print(rviz_config)
#print('Current dir: ', os.getcwd())
def generate_launch_description():

    ans_services = Node(
            package='ans_server',
            executable='cfg_server', 
            parameters=[
                {
                'occ_map': os.path.join(algorithm_data_folder, OCC_FILE_YAML),
                'nav_map': os.path.join(algorithm_data_folder, MAP_FILE_PCD)
                }
            ]) 

    navigator_host = Node(
            package='navigator_host',
            executable='navigator_host', 
        )      

    path_planner = Node(
        	package='path_planner',
        	executable='path_planner',
        	output='screen',
            parameters=[
                {'foward_motion': [0.33, 0.0],
                 'backward_motion': [0.0, 0.0],
                 'left_motion': [0.33, 1.5708/10],
                 'right_motion': [0.33, -1.5708/10],
                 'robot_height': 0.5,
                 'robot_width': 0.4
                 }
            ]
    ) 

    rviz_setup = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config]
    )
          
        
    return launch.LaunchDescription([
        ans_services,
        navigator_host,
        path_planner,
        rviz_setup
   ])

