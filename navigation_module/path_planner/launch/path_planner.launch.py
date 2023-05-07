from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    path_planner = Node(
        	package='path_planner',
        	executable='path_planner',
        	output='screen',
            parameters=[
                {'foward_motion': [0.15, 0.0],
                 'backward_motion': [0.0, 0.0],
                 'left_motion': [0.15, 1.5708/5],
                 'right_motion': [0.15, -1.5708/5],
                 'robot_height': 0.3048,
                 'robot_width': 0.127
                 }
            ]
    ) 
    
    return LaunchDescription([
        path_planner  
    ])