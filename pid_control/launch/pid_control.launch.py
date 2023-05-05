import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node



#CHANGE THIS TO YOUR LOCATION OF THE SAVE DIRECTORY
save_dir='/media/storage'

imu_params_config = os.path.join(get_package_share_directory('imu_pkg'), 'config', 'imu_params.yaml')



def generate_launch_description():
    return launch.LaunchDescription([
        #This is used if you plan to control the robot directly from the
        #vnc server. If not you likely want to start the teleop_twist_keyboard
        #from the controlling node
        #Node(
        #    package='teleop_twist_keyboard',
        #    executable='teleop_twist_keyboard',
        #    output='screen',
        #    prefix='xterm -e'),
        Node(
            package='pid_control',
            executable='pid_control'),
           Node(
            package='imu_pkg',
            executable='imu_node',
            name='imu_node',
            parameters = [imu_params_config]
        ),
        Node(
            package='servo_pkg',
            namespace='servo_pkg',
            executable='servo_node',
            name='servo_node'
        )


        ])

