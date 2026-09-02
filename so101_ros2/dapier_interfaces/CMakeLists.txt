cmake_minimum_required(VERSION 3.16)
project(dapier_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ResearchControlIntent.msg"
  "msg/LocalizationEstimate.msg"
  "msg/ReplanAcknowledgement.msg"
  "msg/InterlockState.msg"
  "msg/SafeCommand.msg"
  "msg/SafetyStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
