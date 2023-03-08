#for general ros stuff
source /opt/ros/foxy/setup.bash

#For AWS Specific stuff (note: the installed version doesn't have a lot
# of the source code required for building from it) which is why this is 
#commented out!
#source /opt/aws/deepracer/lib/setup.bash

#source the main package this is associated with:
source ssh_driver/install/setup.bash


#THE ENVIRONAMENT VARIABLES REQURIED FOR ROS

export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=23

printenv | grep ROS

#ROS DEPENDENCIES AND CHANGING VARIABLES FOR THAT...
#this is done for the reason above, that the aws stuff doesn't have the 
#source code to actually depend on them, you'll want to add things to this
#as you need them
export CMAKE_PREFIX_PATH=/root/deepracer_ws/aws-deepracer-interfaces-pkg/install
#Additinally need to source this:
source /root/deepracer_ws/aws-deepracer-interfaces-pkg/install/setup.bash
echo $CMAKE_PREFIX_PATH



#Servo pkg
source /opt/aws/deepracer/lib/servo_pkg/share/servo_pkg/local_setup.bash
