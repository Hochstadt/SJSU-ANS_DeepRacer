import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node
import os
#Assumes working from the main SJSU repo
rviz_config_file_name = os.path.join('rviz_configs', 'project_presentation.rviz')
repo_locs = ['', '..', os.path.join('..', '..'), os.path.join('..','..','..')]
for i in repo_locs:
    if os.path.exists(os.path.join(i, rviz_config_file_name)):
        rviz_config = os.path.join(i, rviz_config_file_name)
        break

print(rviz_config)
#print('Current dir: ', os.getcwd())
def generate_launch_description():
    

    navigator_host = Node(
            package='navigator_host',
            executable='navigator_host', 
            parameters=[
                {
                    'map_file': 'map_file.pickle'
                }
            ])      

    rviz_setup = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config]
    )
          
        #Node(
        #    package='rviz_interface',
        #    executable='rviz_interface'
        ##),
        
    return launch.LaunchDescription([
        navigator_host,
        rviz_setup
   ])

