cmake_minimum_required(VERSION 3.5)
project(safety)

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

add_executable(e_brake src/e_brake.cpp)
  ament_target_dependencies(e_brake rclcpp 
    std_msgs 
    sensor_msgs 
    nav_msgs)

install(TARGETS
  e_brake
  DESTINATION lib/${PROJECT_NAME})

ament_package()