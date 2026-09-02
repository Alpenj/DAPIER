cmake_minimum_required(VERSION 3.16)
project(dapier_safety_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(dapier_interfaces REQUIRED)
find_package(dapier_localization_core REQUIRED)
find_package(dapier_safety_core REQUIRED)
find_package(dapier_so101_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(dapier_safety_bridge_node
  src/safety_bridge_node.cpp
)
ament_target_dependencies(
  dapier_safety_bridge_node
  ament_index_cpp
  dapier_interfaces
  dapier_localization_core
  dapier_safety_core
  dapier_so101_core
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

install(TARGETS dapier_safety_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
