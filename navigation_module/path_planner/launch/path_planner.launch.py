from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    path_planner = Node(
        	package='path_planner',
        	executable='path_planner',
        	output='screen',
            parameters=[
                {'motions': [0.5, 0.0, 0.5, 1.5708/2, 0.5, -1.5708/2],
                 'robot_height': 0.3048,
                 'robot_width': 0.127
                 }
            ]
    ) 
    
    return LaunchDescription([
        path_planner  
    ])