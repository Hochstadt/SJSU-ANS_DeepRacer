cmake_minimum_required(VERSION 3.5)
project(lidar_scan_acq)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find ROS package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  laser_geometry
  tf2
)

add_executable(lidar_scan_acq src/lidar_scan_acq.cpp)

ament_target_dependencies(lidar_scan_acq ${dependencies})

install(TARGETS lidar_scan_acq
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )


ament_package()
