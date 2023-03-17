import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        #This is used if you plan to control the robot directly from the
        #vnc server. If not you likely want to start the teleop_twist_keyboard
        #from the controlling node
        Node(
            package='ssh_controller',
            executable='ssh_controller'),
        Node(
            package='web_video_server',
            executable='web_video_server',
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e')
   ])

