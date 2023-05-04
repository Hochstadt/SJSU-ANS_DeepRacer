from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        	package='localization',
        	executable='localization',
        	output='screen',
            parameters=[
                {'downsampleMap': True,
                 'downsampleValue': 0.25,
                 'coarse_fit_thresh': 0.25,
                 'fine_fit_thresh': 0.25,
                 'icp_max_corr_dist': 1.0,
                 'icp_max_iters': 100,
                 'min_search_x': -10.0,
                 'max_search_x':  2.0,
                 'delta_search_x': 5,
                 'min_search_y': -2.0,
                 'max_search_y':  10.0,
                 'delta_search_y': 5,
                 'delta_search_ang': 5,
                 'rotationCheckThreshold': 0.017,
                 'positionCheckThrehsold': 0.100,
                 }
            ]
        )
    ])
