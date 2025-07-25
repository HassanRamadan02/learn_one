cmake_minimum_required(VERSION 3.5)
project(learn_one)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
#find_package(geometry_msgs REQUIRED)
#find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(std_msgs REQUIRED)
#find_package(action_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY config description launch worlds src scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/nav_through_poses.py
  scripts/robot_navigator.py
  scripts/angle_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



#install(PROGRAMS learn_one/scripts/covariance_splitter.py
#  DESTINATION lib/${PROJECT_NAME}
#)

#install(TARGETS learn_one/launch/launch_sim.launch.py
#  DESTINATION lib/${PROJECT_NAME}
#)

ament_package()
