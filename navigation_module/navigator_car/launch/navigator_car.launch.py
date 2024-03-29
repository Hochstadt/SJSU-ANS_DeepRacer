import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node
from launch import LaunchDescription

bIMU = False
def generate_launch_description():

    navigator_car = Node(
        package='navigator_car',
        executable='navigator_car'
        )
    
    
    localization = Node(
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
                'min_search_x': 0.0,
                'max_search_x':  8.5,
                'delta_search_x': 5,
                'min_search_y': 3.0,
                'max_search_y':  10.0,
                'delta_search_y': 5,
                'delta_search_ang': 5,
                'rotationCheckThreshold': 0.017,
                'positionCheckThrehsold': 0.100,
                }
        ]
    ) 
    rplidar_node = Node(
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
    )

    lidar_scan_acq = Node(
        package='lidar_scan_acq',
        executable='lidar_scan_acq',
        output='screen',
        parameters=[
            {'saveData': False,
                'maxRange': 12.0}
        ]
    )

    
    vel_controller = Node(
        package='vel_controller',
        executable='vel_controller',
        parameters=[{
                'bIMU': bIMU,
                'velKp': .3,
                'velKi': 0.0001,
                'velKd': 0.001,
                'rotKp': .7,
                'rotKi': 0.01,
                'rotKd': 0.009
            }]            
        )
    servo_pkg = Node(
        package='servo_pkg',
        namespace='servo_pkg',
        executable='servo_node',
        name='servo_node'
        )
    imu_pkg = Node(
        package='imu_pkg',
        executable='imu_node'
    )
    cam_streamer = Node(
        package='camera_streamer',
        executable='camera_streamer'
    )

    if bIMU == True:
        return LaunchDescription([
            navigator_car,
            localization,
            rplidar_node,
            lidar_scan_acq,
            vel_controller,
            servo_pkg,
            imu_pkg,
            cam_streamer
        ])
    else:
        return LaunchDescription([
            navigator_car,
            localization,
            rplidar_node,
            lidar_scan_acq,
            vel_controller,
            servo_pkg,
            cam_streamer
        ])
