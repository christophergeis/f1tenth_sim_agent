cmake_minimum_required(VERSION 3.5)
project(utilities)

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
find_package(agent_interfaces REQUIRED)

add_executable(subscriber_gen src/subscriber_gen.cpp)
  ament_target_dependencies(subscriber_gen rclcpp
    agent_interfaces)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
subscriber_gen
  DESTINATION lib/${PROJECT_NAME})

ament_package()