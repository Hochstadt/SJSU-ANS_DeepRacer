#!/usr/bin/env bash 

CUR_PATH=`pwd`
DEP_PATH="$CUR_PATH/deepracer_deps"
INF_PATH="$DEP_PATH/aws-deepracer-interfaces-pkg"
bError=0

#Default ros2 stuff
if [ ! -d "/opt/ros/foxy" ]
then
  echo "No ros installation detected, go install it. Specifcially the foxy version. This will likely fail somewhere now...."
  bError=1
fi

source /opt/ros/foxy/setup.bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=22
#NO matter what car vs. host cpu we need the following
##INterfaces
if [ ! -d $DEP_PATH ]
then 
  mkdir $DEP_PATH
fi
cd $DEP_PATH

#Check if this is already built:
if [ ! -d "$INF_PATH/build" ]
then
  #Check if deepracer_interface already exists
  if [ ! -d $INF_PATH ]
  then
    git clone git@github.com:aws-deepracer/aws-deepracer-interfaces-pkg.git
  fi
  
  #Go check dependencies and do build
  cd $INF_PATH && rosdep install -i --from-path . --rosdistro foxy -y 
  #Go build the thing
  cd $INF_PATH && colcon build --packages-select deepracer_interfaces_pkg 
fi
source $INF_PATH/install/local_setup.bash
export CMAKE_PREFIX_PATH="$INF_PATH/install"
cd $CUR_PATH

#######################################################
## NOW CHECK IF CAR VS. HOST
#######################################################
if [ -z $1 ]
then
  echo "NO command line given, no things will be built beyond the $INF_PATH"
elif [ $1 = "car" ]
then
  ##########################################################3
  ## CAR
  ###########################################################
  echo "Car build selected."
  #Relevant paths
  CAM_PATH="$DEP_PATH/aws-deepracer-camera-pkg/camera_pkg"
  LIDAR_PATH="$DEP_PATH/rplidar_ros"
  AWS_PATH="/opt/aws/deepracer/lib"
  SSH_DRIVER_PATH="$CUR_PATH/ssh_driver"
  
  cd $DEP_PATH

  # Camera
  ###########################################################
  if [ ! -d "$CAM_PATH/build" ]
  then
    #Check if already cloned
    if [ ! -d $CAM_PATH ]
    then 
      echo "Cloning deepracer camera package"
      git clone git@github.com:aws-deepracer/aws-deepracer-camera-pkg.git
    fi
    echo "Building camera package"
    cd $CAM_PATH && colcon build --packages-select camera_pkg
  else
    echo "Camera package already exists and is built"
  fi
  source $CAM_PATH/install/setup.bash

  cd $DEP_PATH
  
  # rplidar 
  #############################################################
  #Check if built
  if [ ! -d "$LIDAR_PATH/build" ]
  then
    #Check if cloned
    if [ ! -d $LIDAR_PATH ]
    then
      echo "Cloning rplidar package"
      git clone -b ros2 git@github.com:Slamtec/rplidar_ros.git
    fi
    echo "Building rplidar package"
    cd $LIDAR_PATH && colcon build
  else
    echo "Lidar package already exists and is built"
  fi
  source $LIDAR_PATH/install/setup.bash

  cd $DEP_PATH
  #Add additional dependencies here

  #AWS
  ###########################################################3
  ##IF on car we can take advantage of the AWS installation, These are not the
  #full blown source code, just the header files and compiled binaries
  if [ -d $AWS_PATH ]
  then
    echo "Sourcing AWS Dependencies"
    #source the servo package
    source /opt/aws/deepracer/lib/servo_pkg/share/servo_pkg/local_setup.bash
    #source any other aws related stuff here
  else
    echo "Error: Did not detect AWS installations. Are you are on the correct system (ie. not the host)?" 
    bError=1
  fi
 

  #ssh_driver pkg
  ##########################################################
  cd $CUR_PATH
  if [ $bError != 1 ]
  then
    if [ ! -d $SSH_DRIVER_PATH/build ]
    then
        echo "Building ssh_driver package"
        cd $SSH_DRIVER_PATH && colcon build
    fi
    source $SSH_DRIVER_PATH/install/local_setup.bash
  fi
    
elif [ $1 = "host" ]
then
  ####################################################################
  ## HOST
  ####################################################################
  echo "Host build selected."
  #Paths
  WVS_PATH="$DEP_PATH/web_video_server"
  SSH_CONTROLLER_PATH="$CUR_PATH/ssh_controller"

  #Web video server
  #foxy ros does not have web_video_server this is required...
  cd $DEP_PATH
  if [ ! -d $WVS_PATH ]
  then
    git clone -b ros2 git@github.com:RobotWebTools/web_video_server.git
  fi
  #Check if already built
  if [ ! -d "$WVS_PATH/build" ]
  then
    echo "building web_video_server"
    cd $WVS_PATH && colcon build
  else
    echo "Web_video_server package already exists and is built"
  fi
  source $WVS_PATH/install/local_setup.bash

  #ssh_controller
  cd $CUR_PATH
  if [ ! -d "$SSH_CONTROLLER_PATH/build" ]
  then
    echo "building ssh_controller"
    cd $SSH_CONTROLLER_PATH && colcon build
  else
    echo "ssh_controller already built"
  fi
  source $SSH_CONTROLLER_PATH/install/local_setup.bash
  
else
  echo "'$1' is not an understood argument, try one of the following"
  echo "source setup/sourceme.sh car"
  echo "source setup/sourceme.sh host"
fi

if [ $bError != 1 ]
then
  echo "Packages successfult built and installed."
  echo "CMAKE_PREFIX_PATH: "
  echo $CMAKE_PREFIX_PATH
  echo "ROS environment variables: "
  printenv | grep ROS
else
  echo "Package building/install failed."
fi

cd $CUR_PATH



#No matter what car vs. host cpu we need the following

#Deepracer interfaces package





