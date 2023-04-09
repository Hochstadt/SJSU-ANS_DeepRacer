# To setup simulator:
## On DeepRacer or Linux Computer:
0. Ensure ROS Foxy is installed on local computer

1. Install Gazebo and XACRO package

```
>>sudo apt install ros-foxy-gazebo-ros-pkgs
>>sudo apt install ros-foxy-xacro
```

2. Source ROS Foxy

```>>source /opt/ros/foxy/setup.bash```

3. Create Directory for Repo

```>>mkdir <checkout_directory>```

4. Checkout Git Repo and navigate to simulation build directory:

```
>>git clone git@github.com:Hochstadt/SJSU-ANS_DeepRacer.git -b  simulation
>>cd <checkout directory>/SJSU-ANS_DeepRacer/simulator
```

5. Build DeepRacer Simulator Package

```>>colcon build --symlink-install```

6. Source DeepRacer Simulator Package

```>>source ./install/setup.bash```

7. Start Simulation

```>>ros2 launch deepracer_sim launch_sim.launch.py```

## Setup Building Model in Simulation:

0. In Gazebo window, Click on "Insert" tab on the left panel

1. Click "Add Path" button on the left panel

2. Navigate to <checkout directory>/SJSU-ANS_DeepRacer/simulator/deepracer_sim/models
  
3. Click the "Choose" button
  
4. Select "asymmetric_room" on the left panel and place model on the map
