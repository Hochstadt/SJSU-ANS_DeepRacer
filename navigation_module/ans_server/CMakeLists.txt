cmake_minimum_required(VERSION 3.8)
project(ans_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find system dependencies


## Find ROS package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(ans_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_map_server
  tf2
  ans_msgs
)

add_executable(cfg_server src/ans_cfg_services.cpp)
ament_target_dependencies(cfg_server ${dependencies} nav_msgs)

ament_export_dependencies(rosidl_default_runtime)


install(TARGETS 
  cfg_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )
 

ament_package()
