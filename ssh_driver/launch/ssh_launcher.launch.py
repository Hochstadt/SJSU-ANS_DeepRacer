import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node

#CHANGE THIS TO YOUR LOCATION OF THE SAVE DIRECTORY
save_dir='/media/storage'

# Either the ssh_driver or cmdvel_to_servo_pkg handle cmd_vel -> servo_msg
servo_msg_publisher='cmdvel_to_servo_pkg'
# servo_msg_publisher='ssh_driver'

required_nodes = [
    #This is used if you plan to control the robot directly from the
    #vnc server. If not you likely want to start the teleop_twist_keyboard
    #from the controlling node
    #Node(
    #    package='teleop_twist_keyboard',
    #    executable='teleop_twist_keyboard',
    #    output='screen',
    #    prefix='xterm -e'),
    Node(
        package='ssh_driver',
        executable='ssh_driver'),
    Node(
        package='servo_pkg',
        namespace='servo_pkg',
        executable='servo_node',
        name='servo_node'
    ),
    #Add nodes needed for data collection
    Node(
        package='rplidar_ros2',
        executable='rplidar_scan_publisher',
        name='rplidar_scan_publisher',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }]
    ),
    Node(
        package='data_collector',
        executable='data_collector',
        name='data_collector',
        parameters=[{
            'save_dir':save_dir}]
    )
]

# Only launch cmdvel_to_servo node if we're using it to publish servo messages
if servo_msg_publisher == "cmdvel_to_servo_pkg":
    required_nodes.append(Node(
        package='cmdvel_to_servo_pkg',
        namespace='cmdvel_to_servo_pkg',
        executable='cmdvel_to_servo_node',
        name='cmdvel_to_servo_node'
    ))

def generate_launch_description():
    return launch.LaunchDescription(required_nodes)

