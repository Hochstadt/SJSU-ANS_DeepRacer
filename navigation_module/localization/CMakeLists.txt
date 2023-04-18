cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


## Find system dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS core features filters io segmentation surface)

## Find ROS package dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(laser_geometry REQUIRED)

set(dependencies
  pcl_conversions
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_eigen
  laser_geometry
  EIGEN3
  PCL
)

add_executable(localization src/localization.cpp)
target_link_libraries(localization ${PCL_LIBRARIES})

ament_target_dependencies(localization ${dependencies} geometry_msgs tf2_geometry_msgs tf2_eigen)

install(TARGETS localization
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )

ament_package()
