cmake_minimum_required(VERSION 3.5)
project(differential_drive_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

add_executable(differential_drive_controller src/differential_drive_controller.cpp)


ament_target_dependencies(differential_drive_controller
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  gazebo_ros
)

install(TARGETS
  differential_drive_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
