#!/usr/bin/env bash
#Not super necessary for now
#Check if user wants to remove current build stuff to do a re-build
#if [ "$1" = "rebuild" ]
#then
#  bRebuild=1
#else
#  bRebuild=0
#fi

#Update PATH as per Curtis' readme everything is installed in username/local/.bin
#so have to add that to the path...
#export PATH=$PATH:/home/deepracer/.local/bin

CUR_PATH=`pwd`
DEP_PATH="$CUR_PATH/deepracer_deps"
INF_PATH="$DEP_PATH/aws-deepracer-interfaces-pkg"
ANS_INTERFACES_PATH="$CUR_PATH/ans_interfaces"
bError=0
bLC=1

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
ROS_NUMPY="$DEP_PATH/ros2_numpy"





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

cd $DEP_PATH
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

cd $DEP_PATH
# ros2numpy
########################################################################3
#Check if built
if [ ! -d "$ROS_NUMPY/build" ]
then
  if [ ! -d $ROS_NUMPY ]
  then
    echo "Cloning common interfaces"
    git clone -b foxy-devel git@github.com:taylormaurer4323/ros2_numpy.git
  fi
  echo "Building ros2 numpy"
  cd $ROS_NUMPY && colcon build
else
  echo "Common interfaces packages already exists and is built"
fi
source $ROS_NUMPY/install/setup.bash


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
  IMU_PATH="$DEP_PATH/aws-deepracer-imu-pkg/imu_pkg"
  LIDAR_PATH="$DEP_PATH/rplidar_ros"
  AWS_PATH="/opt/aws/deepracer/lib"
  IMU_PKG="$DEP_PATH/larsll-deepracer-imu-pkg/imu_pkg"
  AWS_DEEPRACER_NAV="$DEP_PATH/aws-deepracer"
  ROS_NUMPY="$DEP_PATH/ros2_numpy"

  #Paths for custom packages
  SSH_DRIVER_PATH="$CUR_PATH/ssh_driver"
  PID_CONTROL_PATH="$CUR_PATH/pid_control"
  DATA_COL_PATH="$CUR_PATH/data_collector"
  LOCALIZER_PATH="$CUR_PATH/navigation_module/localization"
  LIDARACQ_PATH="$CUR_PATH/navigation_module/lidar_scan_acq"
  NAVIGATOR_CAR="$CUR_PATH/navigation_module/navigator_car"
  CONTROLLER_PATH="$CUR_PATH/navigation_module/vel_controller"
  CAM_STREAMER="$CUR_PATH/navigation_module/camera_streamer"

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

  if [ $bLC != 1 ]
  then
    cd $DEP_PATH
    if [ ! -d "$IMU_PATH/build" ]
    then
      #Check if already cloned
      if [ ! -d $IMU_PATH ]
      then
        echo "Cloning deepracer imu package"
        git clone https://github.com/robofoundry/aws-deepracer-imu-pkg.git
      fi
      echo "Changing device address to possible new standard"
      sed -i 's/105/104/' $IMU_PATH/config/imu_params.yaml
      echo "Changing rate to 100"
      sed -i 's/60/100/' $IMU_PATH/config/imu_params.yam
      echo "Building imu package"
      cd $IMU_PATH && colcon build
    else
      echo "imu package already exists and is built"
    fi
    source $IMU_PATH/install/local_setup.bash
  fi

  cd $DEP_PATH


  # AWS DeepRacer ROS Navigation Stack
  # -- Need cmdvel_to_servo_pkg for joystick control
  #############################################################
  #Check if built
  if [ ! -d "$AWS_DEEPRACER_NAV/build" ]
  then
    #Check if cloned
    if [ ! -d $AWS_DEEPRACER_NAV ]
    then
      echo "AWS DeepRacer Navigation Stack repo"
      git clone https://github.com/aws-deepracer/aws-deepracer.git
      # Replace  constants.ACTION_PUBLISH_TOPIC  with  'ctrl_pkg/servo_msg'
      sed -i "s/constants.ACTION_PUBLISH_TOPIC/'\/ctrl_pkg\/servo_msg'/" aws-deepracer/deepracer_nodes/cmdvel_to_servo_pkg/cmdvel_to_servo_pkg/cmdvel_to_servo_node.py
    fi
    echo "Building cmdvel_to_servo_pkg package"
    cd $AWS_DEEPRACER_NAV && colcon build --packages-select cmdvel_to_servo_pkg
  else
    echo "cmdvel_to_servo_pkg package already exists and is built"
  fi
  source $AWS_DEEPRACER_NAV/install/local_setup.bash

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

  #IMU PKG
  ###########################################################3
  if [ $bLC != 0 ]
    then
    cd $DEP_PATH
    if [ ! -d "$IMU_PKG/build" ]
    then
      #Check if cloned
      if [ ! -d $IMU_PKG ]
      then
        echo "Cloning IMU PKG"
        git clone git@github.com:taylormaurer4323/larsll-deepracer-imu-pkg.git
      fi
      echo "BUilding imu package"
      echo "-------------------------------------------------"
      echo "WARNING: FOR THE PACKAGE TO BUILD CORRECTLY YOU NEED TO INSTALL BMI160-i2c and smbus2"
      echo "TO DO SO - RUN 'pip install BMI160-i2c smbus2'"
      cd $IMU_PKG && colcon build
    else
      echo "IMU package already exists and is built"
    fi
    source $IMU_PKG/install/local_setup.bash
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
    echo "sourcing ssh_driver"
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
    echo "sourcing data collector"
    source $DATA_COL_PATH/install/setup.bash
  fi
  #localizer
  #################################################################
  cd $CUR_PATH
  if [ $bError != 1 ]
  then
    if [ ! -d $LOCALIZER_PATH/build ]
    then
      echo "Building localization package"
      cd $LOCALIZER_PATH && colcon build
    fi
    echo "sourcing localizer"
    source $LOCALIZER_PATH/install/setup.bash
  fi

  #lidar_scan_acq
  #################################################################
  cd $CUR_PATH
  if [ $bError != 1 ]
  then
    if [ ! -d $LIDARACQ_PATH/build ]
    then
      echo "Building lidar acquisition package"
      cd $LIDARACQ_PATH && colcon build
    fi
    echo "sourcing lidar acq"
    source $LIDARACQ_PATH/install/setup.bash
  fi

  #controller
  #################################################################
  if [ $bLC != 0 ]
  then
    cd $CUR_PATH
    if [ $bError != 1 ]
    then
      if [ ! -d $CONTROLLER_PATH/build ]
      then
        echo "Building controller package"
        cd $CONTROLLER_PATH && colcon build
      fi
      echo "Sourcing controller"
      source $CONTROLLER_PATH/install/setup.bash
    fi
  fi

  #udp_sender pkg
  ##########################################################
  if [ $bLC != 1 ]
  then
    cd $CUR_PATH
    if [ $bError != 1 ]
    then
      if [ ! -d $PID_CONTROL_PATH/build ]
      then
          echo "Building pid_control package"
          cd $PID_CONTROL_PATH && colcon build
      fi
      echo "sourcing controller"
      source $PID_CONTROL_PATH/install/setup.bash
    fi
  fi
  
  #navigator car
  ##########################################################
  cd $CUR_PATH
  if [ $bError != 1 ]
  then
    if [ ! -d $NAVIGATOR_CAR/build ]
    then
      echo "Building navigator car package"
      cd $NAVIGATOR_CAR && colcon build
    fi
    echo "Sourcing navigator_car"
    source $NAVIGATOR_CAR/install/setup.bash
  fi

  #camera streamer
  ##########################################################
  cd $CUR_PATH
  if [ $bError != 1 ]
  then
    if [ ! -d $CAM_STREAMER/build ]
    then
      echo "Building camera streamer package"
      cd $CAM_STREAMER && colcon build
    fi
    echo "Sourcing camera_streamer"
    source $CAM_STREAMER/install/setup.bash
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
  ANS_SERVER="$CUR_PATH/navigation_module/ans_server"
  ANS_MSGS="$CUR_PATH/navigation_module/ans_msgs"
  ANS_GUI_PATH="$CUR_PATH/ans_gui"
  NAVIGATOR_HOST="$CUR_PATH/navigation_module/navigator_host"
  PATH_PLANNER="$CUR_PATH/navigation_module/path_planner"

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

  cd $CUR_PATH

  #ans_msgs
  ####################################################
  if [ ! -d "$ANS_MSGS/build" ]
  then
    echo "building ans_msgs"
    cd $ANS_MSGS && colcon build
  else
    echo "ans_msgs already built"
  fi
  source $ANS_MSGS/install/setup.bash

  cd $CUR_PATH
  #ans_server
  ####################################################
  if [ ! -d "$ANS_SERVER/build" ]
  then
    echo "building ans_server"
    cd $ANS_SERVER && colcon build
  else
    echo "ans_server already built"
  fi
  source $ANS_SERVER/install/setup.bash


  #path_planner
  ##########################################33
    
  cd $CUR_PATH
  if [ ! -d "$PATH_PLANNER/build" ]
  then
    echo "building path planner"
    cd $PATH_PLANNER && colcon build
  else
    echo "path planner host already built"
  fi
  source $PATH_PLANNER/install/setup.bash


  #navigator_host
  ##########################################33

  cd $CUR_PATH
  if [ ! -d "$NAVIGATOR_HOST/build" ]
  then
    echo "building navigator host"
    cd $NAVIGATOR_HOST && colcon build
  else
    echo "navigator host already built"
  fi
  source $NAVIGATOR_HOST/install/setup.bash

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





