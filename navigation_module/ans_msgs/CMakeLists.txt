cmake_minimum_required(VERSION 3.8)
project(ans_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find ROS package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FilePath.srv"
  "srv/Pose.srv"
  "srv/StartFlag.srv"
 )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
