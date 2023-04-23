# To setup ans_msgs and ans_servers:
## On DeepRacer or Linux Computer:
0. Ensure ROS Foxy is installed on local computer

1. Install Navigation2 package

```
>>sudo apt install ros-humble-navigation2
```

2. Source ROS Foxy

```
>>source /opt/ros/foxy/setup.bash
```

3. Create Directory for Repo

```
>>mkdir <checkout_directory>
```

4. Checkout Git Repo and navigate to simulation build directory:

```
>>git clone git@github.com:Hochstadt/SJSU-ANS_DeepRacer.git
>>cd <checkout directory>/SJSU-ANS_DeepRacer/navigation_module
```

5. Build DeepRacer Simulator Package

```
>>colcon build --packages-select ans_msgs ans_services
```

6. Source DeepRacer Simulator Package

```
>>source ./install/setup.bash
```
  
# To run ans_servers:
## On DeepRacer or Linux Computer:
0. Source Directories
  
```
>>source /opt/ros/foxy/setup.bash
>>source ./install/setup.bash
```
  
1. Start ans_server

```
>>ros2 run ans_server cfg_server
```

# To load point cloud map, occupancy map and goal state:
## On DeepRacer or Linux Computer:
0. In new terminal window, source Directories
  
```
>>source /opt/ros/foxy/setup.bash
>>source ./install/setup.bash
```

1. Run cfg_client with point cloud map
   Note: The input is a filepath to a PCD file.

```
>>ros2 run ans_services cfg_client load_nav_map test_smaller.pcd
```

2. Run cfg_client with occupancy map
   Note: The input is a filepath to a yaml file.
  
```
>>ros2 run ans_services cfg_client load_occupancy_map occupancy_map.yaml
```
  
3. Run cfg_client with occupancy map
   Note: The input is pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w.

```
>>ros2 run ans_server cfg_client load_goal_state 0 0 0 0 0 0 1
```
  
