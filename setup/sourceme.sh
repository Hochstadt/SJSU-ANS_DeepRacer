#!/usr/bin/env bash

CUR_PATH=`pwd`
DEP_PATH="$CUR_PATH/deepracer_deps"
INF_PATH="$DEP_PATH/aws-deepracer-interfaces-pkg"
ANS_INTERFACES_PATH="$CUR_PATH/ans_interfaces"
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
#Additional car & host dependencies
LASER_GEOM_PATH="$DEP_PATH/laser_geometry"
COMMON_INF_PATH="$DEP_PATH/common_interfaces"
cd $DEP_PATH
# laser_geom
####################################################################
#Check if built
if [ ! -d "$LASER_GEOM_PATH/build" ]
then
  #Check if cloned
  if [ ! -d $LASER_GEOM_PATH ]
  then
    echo "Cloning laser geometry"
    git clone -b ros2 git@github.com:ros-perception/laser_geometry.git
  fi
  echo "Building laser geometry"
  cd $LASER_GEOM_PATH && colcon build
else
  echo "Laser geometry package already exists and is built"
fi
source $LASER_GEOM_PATH/install/local_setup.bash
cd $DEP_PATH
# common_interfaces
########################################################################3
#Check if built
if [ ! -d "$COMMON_INF_PATH/build" ]
then
  if [ ! -d $COMMON_INF_PATH ]
  then
    echo "Cloning common interfaces"
    git clone -b foxy git@github.com:ros2/common_interfaces.git
  fi
  echo "Building common interfaces this will take ~10 minutes on deepracer"
  cd $COMMON_INF_PATH && colcon build
else
  echo "Common interfaces packages already exists and is built"
fi
source $COMMON_INF_PATH/install/setup.bash

# ans_interfaces
########################################################################3
#Check if built
if [ ! -d "$ANS_INTERFACES_PATH/build" ]
then

  echo "Building ans_interfaces"
  cd $ANS_INTERFACES_PATH && colcon build
else
  echo "ans_interfaces already exists and is built"
fi
source $ANS_INTERFACES_PATH/install/setup.bash

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
  #Relevant paths for dependencies
  CAM_PATH="$DEP_PATH/aws-deepracer-camera-pkg/camera_pkg"
  LIDAR_PATH="$DEP_PATH/rplidar_ros"
  AWS_PATH="/opt/aws/deepracer/lib"
  AWS_DEEPRACER_NAV="$DEP_PATH/aws-deepracer"

  #Paths for custom packages
  SSH_DRIVER_PATH="$CUR_PATH/ssh_driver"
  DATA_COL_PATH="$CUR_PATH/data_collector"
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
  source $CAM_PATH/install/local_setup.bash

  cd $DEP_PATH

  # rplidar
  #############################################################
  #Check if built
  if [ ! -d "$AWS_DEEPRACER_NAV/build" ]
  then
    #Check if cloned
    if [ ! -d $AWS_DEEPRACER_NAV ]
    then
      echo "AWS DeepRacer Navigation Stack repo"
      git clone https://github.com/aws-deepracer/aws-deepracer.git
      # For some reason the topic they publish is /cmdvel_to_servo_node/servo_msg, it needs to be /ctrl_pkg/servo_msg
      # Replace  constants.ACTION_PUBLISH_TOPIC  with  'ctrl_pkg/servo_msg'
    fi
    echo "Building cmdvel_to_servo_pkg package"
    cd $AWS_DEEPRACER_NAV && colcon build --packages-select cmdvel_to_servo_pkg
  else
    echo "cmdvel_to_servo_pkg package already exists and is built"
  fi
  source $AWS_DEEPRACER_NAV/install/local_setup.bash

  cd $DEP_PATH

  # AWS DeepRacer ROS Navigation Stack
  # -- Need cmdvel_to_servo_pkg for joystick control
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
  source $LIDAR_PATH/install/local_setup.bash


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
    source $SSH_DRIVER_PATH/install/setup.bash
  fi

  #data_collector
  ###############################################################
  cd $CUR_PATH
  if [ $bError != 1 ]
  then
    if [ ! -d $DATA_COL_PATH/build ]
    then
      echo "Building data collector package"
      cd $DATA_COL_PATH && colcon build
    fi
    source $DATA_COL_PATH/install/setup.bash
  fi

elif [ $1 = "host" ]
then
  ####################################################################
  ## HOST
  ####################################################################
  echo "Host build selected."
  #Paths
  SSH_CONTROLLER_PATH="$CUR_PATH/ssh_controller"
  RVIZ_INF="$CUR_PATH/rviz_interface"
  ANS_GUI_PATH="$CUR_PATH/ans_gui"

  #Rviz interface
  cd $CUR_PATH
  if [ ! -d "$RVIZ_INF/build" ]
  then
    echo "building rviz interface"
    cd $RVIZ_INF && colcon build
  else
    echo "rviz interface already built"
  fi
  source $RVIZ_INF/install/local_setup.bash

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

  # ANS GUI
  if [ ! -d "$ANS_GUI_PATH/build" ]
  then
    echo "building ans_gui"
    cd $ANS_GUI_PATH && colcon build
  else
    echo "ans_gui already built"
  fi
  source $ANS_GUI_PATH/install/local_setup.bash

else
  echo "'$1' is not an understood argument, try one of the following"
  echo "source setup/sourceme.sh car"
  echo "source setup/sourceme.sh host"
fi

if [ $bError != 1 ]
then
  echo "Packages successfult built and installed."
  #echo "CMAKE_PREFIX_PATH: "
  #echo $CMAKE_PREFIX_PATH
  #echo "ROS environment variables: "
  #printenv | grep ROS
else
  echo "Package building/install failed."
fi

cd $CUR_PATH



#No matter what car vs. host cpu we need the following

#Deepracer interfaces package





