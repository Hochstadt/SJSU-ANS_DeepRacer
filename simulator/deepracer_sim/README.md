# Note:
This work relies heavily on the work from https://github.com/aws-deepracer/aws-deepracer.
Modification were made to work for our configuration and to be compatible with multiple versions of ros2.

# To setup simulator:
## On DeepRacer or Linux Computer:
0. Ensure ROS Foxy is installed on local computer

1. Install Gazebo and XACRO package

```
>>sudo apt install ros-foxy-gazebo-ros-pkgs
>>sudo apt install ros-foxy-xacro
>>sudo apt install ros-foxy-joint-state-publisher
>>sudo apt install ros-foxy-joint-state-broadcaser
>>sudo apt install ros-foxy-ros2-controllers
>>sudo apt install ros-foxy-ros2-controller-manager
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
>>cd <checkout directory>/SJSU-ANS_DeepRacer/simulator
```

5. Build DeepRacer Simulator Package

```
>>colcon build --symlink-install
```

6. Source DeepRacer Simulator Package and Gazebp

```
>>source ./install/setup.bash
>>source /usr/share/gazebo/setup.sh
```

7. Start Simulation

```
>>ros2 launch deepracer_bringup deepracer_sim.launch.py world:=./deepracer_sim/worlds/asymmetric.world
```
