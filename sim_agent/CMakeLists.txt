cmake_minimum_required(VERSION 3.5)
project(sim_agent)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
# find_package(agent_interfaces REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)

add_executable(e_brake src/e_brake.cpp)
  ament_target_dependencies(e_brake rclcpp 
    std_msgs 
    sensor_msgs 
    nav_msgs 
    ackermann_msgs)

add_executable(wall_follow src/wall_follow.cpp)
  ament_target_dependencies(wall_follow rclcpp 
    std_msgs 
    sensor_msgs 
    nav_msgs 
    ackermann_msgs)

# add_executable(wall_follow_scan_tf2 src/wall_follow_scan_tf2.cpp)
#   ament_target_dependencies(wall_follow_scan_tf2 rclcpp 
#     geometry_msgs
#     tf2_ros
#     agent_interfaces)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  e_brake
  wall_follow
#  wall_follow_scan_tf2
  DESTINATION lib/${PROJECT_NAME})

ament_package()