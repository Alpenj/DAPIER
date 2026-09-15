cmake_minimum_required(VERSION 3.16)
project(dapier_safety_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(dapier_localization_core REQUIRED)
find_package(dapier_so101_core REQUIRED)

add_library(dapier_safety_core SHARED
  src/safety_controller.cpp
)
target_include_directories(dapier_safety_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  dapier_safety_core
  dapier_localization_core
  dapier_so101_core
)

install(
  TARGETS dapier_safety_core
  EXPORT export_dapier_safety_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_dapier_safety_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(dapier_safety_core)
ament_export_dependencies(dapier_localization_core dapier_so101_core)

if(BUILD_TESTING)
  enable_testing()
  add_executable(
    test_safety_controller
    test/safety_controller_smoke.cpp
  )
  target_link_libraries(test_safety_controller dapier_safety_core)
  add_test(NAME test_safety_controller COMMAND test_safety_controller)
endif()

ament_package()
