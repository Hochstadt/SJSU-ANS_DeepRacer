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
        output='screen',
        parameters=[
                {'icp_fit_thresh': 0.01,
                 'gicp_fit_thresh': 0.01,
                 'icp_trans_eps': 1e-8,
                 'icp_max_corr_dist': 1000.0,
                 'icp_max_iters': 100,
                 'min_search_x': -12.0,
                 'max_search_x':  12.0,
                 'delta_search_x': 5,
                 'min_search_y': -12.0,
                 'max_search_y':  12.0,
                 'delta_search_y': 5,
                 'delta_search_ang': 5,
                 'rotationCheckThreshold': 0.017,
                 'positionCheckThrehsold': 0.010
                 }
            ]
    ) 
    
     return LaunchDescription([
        localization
    ])
