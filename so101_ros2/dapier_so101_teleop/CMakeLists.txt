cmake_minimum_required(VERSION 3.16)
project(dapier_so101_teleop)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(dapier_so101_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(safe_leader_follower src/safe_leader_follower.cpp)
ament_target_dependencies(
  safe_leader_follower
  dapier_so101_core
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)

install(TARGETS safe_leader_follower DESTINATION lib/dapier_so101_teleop)
install(
  DIRECTORY config launch
  DESTINATION share/dapier_so101_teleop
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
