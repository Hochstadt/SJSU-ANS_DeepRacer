import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node

# The expected filename pattern of the point cloud and occupancy YAML files
pcd_map_filename='*final_map*.pickle'
occ_yml_filename='*occupancy_map*.yaml'

# Insert parameter to have either the ssh_driver or aws_servo_pkg handle cmd_vel -> servo_msg OR

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='ans_gui_pkg',
            executable='ans_gui'),
   ])

