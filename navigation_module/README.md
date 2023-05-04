# To setup navigation module:
## On DeepRacer or Linux Computer:
0. Ensure ROS Foxy is installed on local computer

1. Install required python packages

```
>>pip install open3d
>>pip install ros2_numpy==0.0.2
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

5. Build Navigation Module Packages

```
>>colcon build
```

6. Source Navigation Module Packages

```
>>source ./install/setup.bash
```
 
