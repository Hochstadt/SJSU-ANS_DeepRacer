cmake_minimum_required(VERSION 3.5)
project(lidar_scan_acq)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find system dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS core features filters io segmentation surface)

## Find ROS package dependencies
find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(laser_geometry REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  laser_geometry
  tf2
)

set(dependencies
  pcl_conversions
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  laser_geometry
  EIGEN3
  PCL
)

add_executable(lidar_scan_acq src/lidar_scan_acq.cpp)
target_link_libraries(lidar_scan_acq ${PCL_LIBRARIES})

ament_target_dependencies(lidar_scan_acq ${dependencies})

install(TARGETS lidar_scan_acq
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )


ament_package()
