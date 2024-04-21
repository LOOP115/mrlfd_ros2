cmake_minimum_required(VERSION 3.8)
project(joints_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)

find_package(franka_msgs REQUIRED)
find_package(ctrl_interfaces REQUIRED)

add_executable(joints_trajectory src/joints_trajectory.cpp)
ament_target_dependencies(joints_trajectory rclcpp rclcpp_action std_msgs trajectory_msgs sensor_msgs kdl_parser franka_msgs ctrl_interfaces)


install(TARGETS
  joints_trajectory
  DESTINATION lib/${PROJECT_NAME})


ament_package()
