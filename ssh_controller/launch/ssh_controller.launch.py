import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node
import os
#Assumes working from the main SJSU repo
rviz_config_file = 'rviz_configs/taylor_ssh_controller.rviz'
print('Current dir: ', os.getcwd())
def generate_launch_description():
    return launch.LaunchDescription([
        #This is used if you plan to control the robot directly from the
        #vnc server. If not you likely want to start the teleop_twist_keyboard
        #from the controlling node
        Node(
            package='ssh_controller',
            executable='ssh_controller'),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e'),
        Node(
            package='rviz_interface',
            executable='rviz_interface'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file]
        )
   ])

