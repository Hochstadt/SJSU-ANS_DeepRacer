# To build the ssh driver (as of 3/7)
1. Follow this guide https://github.com/aws-deepracer/aws-deepracer-interfaces-pkg. Specifically the download and build instructions. You want to have a folder created in /root/ called deepracer_ws for this, jsut like the guide says. The aws install that comes with the vehicle does not contain all the source code you need for building other things. This guide will also have you do these things as sudo. You want to stay as sudo for the remainder of this (unless otherwise stated)

2. Once that is done and completely built, source the ssh_driver that should be in the cloned SJSU-ANS_DeepRacer folder. 
```>>source source_ssh_driver.sh```

3. Once this is sourced you can build the ssh driver by moving into the ssh_driver directory, and running `>>colcon build`. This will take a few minutes to build and hopefully is successful! 

# Now for running. 

1. First you want to be in a different terminal than the one you built in. 
Unsure why this is a thing, but ROS says to do so.

2. You want to A) be sudo run (`>>sudo su`), and B) source the ssh_driver files (`>>source source_ssh_driver.sh`)

3. Then you can launch the ssh_driver with the following:
``` ros2 launch ssh_driver ssh_launcher.launch.py```
This will start the servo node and the ssh_driver node that interprets the teleop_twist_keyboard commands and turn them into the commands to the motors and steering. An important thing to remember is that your car won't go anywhere if
the main battery is not turned in.

4. From the computer you want to control things, as sudo, re-source the appropriate
ros items (ie. source /opt/ros/foxy/setup.bash), and then run the teleop
keyboard:
```>>ros2 run teleop_twist_keyboard teleop_twist_keyboard```

Then using this interface you can run the keyboard. Note, when working with multiple machines, the multicast tool came in handy and this thread did as well:
https://github.com/aws-deepracer/aws-deepracer-interfaces-pkg













