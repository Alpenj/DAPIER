cmake_minimum_required(VERSION 3.16)
project(dapier_localization_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

add_library(dapier_localization_core SHARED
  src/localization_runtime.cpp
  src/localization_monitor.cpp
  src/orbslam3_bridge.cpp
)
target_include_directories(dapier_localization_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS dapier_localization_core
  EXPORT export_dapier_localization_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_dapier_localization_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(dapier_localization_core)

if(BUILD_TESTING)
  enable_testing()
  add_executable(
    test_localization_runtime_contract
    test/localization_runtime_contract_smoke.cpp
  )
  target_link_libraries(
    test_localization_runtime_contract
    dapier_localization_core
  )
  add_test(
    NAME test_localization_runtime_contract
    COMMAND test_localization_runtime_contract
  )

  add_executable(
    test_localization_monitor
    test/localization_monitor_smoke.cpp
  )
  target_link_libraries(
    test_localization_monitor
    dapier_localization_core
  )
  add_test(
    NAME test_localization_monitor
    COMMAND test_localization_monitor
  )

  add_executable(
    test_orbslam3_bridge
    test/orbslam3_bridge_smoke.cpp
  )
  target_link_libraries(
    test_orbslam3_bridge
    dapier_localization_core
  )
  add_test(
    NAME test_orbslam3_bridge
    COMMAND test_orbslam3_bridge
  )
endif()

ament_package()
