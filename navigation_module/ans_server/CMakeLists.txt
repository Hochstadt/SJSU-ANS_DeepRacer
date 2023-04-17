cmake_minimum_required(VERSION 3.8)
project(ans_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find system dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS core features filters io segmentation surface)

## Find ROS package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(ans_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
  pcl_conversions
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_map_server
  tf2
  EIGEN3
  PCL
  ans_msgs
)

add_executable(cfg_client src/ans_cfg_client.cpp)
target_link_libraries(cfg_client ${PCL_LIBRARIES})
ament_target_dependencies(cfg_client ${dependencies} nav_msgs)

add_executable(cfg_server src/ans_cfg_services.cpp)
target_link_libraries(cfg_server ${PCL_LIBRARIES})
ament_target_dependencies(cfg_server ${dependencies} nav_msgs)


install(TARGETS 
  cfg_server
  cfg_client
  DESTINATION lib/${PROJECT_NAME})
 
ament_export_dependencies(rosidl_default_runtime)

ament_package()
