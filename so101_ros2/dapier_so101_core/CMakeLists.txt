cmake_minimum_required(VERSION 3.16)
project(dapier_so101_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(dapier_so101_core SHARED
  src/joint_model.cpp
)
target_include_directories(dapier_so101_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(dapier_so101_core PUBLIC yaml-cpp)

install(
  TARGETS dapier_so101_core
  EXPORT export_dapier_so101_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/dapier_so101_core/config)

ament_export_targets(export_dapier_so101_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(dapier_so101_core)
ament_export_dependencies(yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joint_model test/test_joint_model.cpp)
  if(TARGET test_joint_model)
    target_link_libraries(test_joint_model dapier_so101_core)
    target_compile_definitions(
      test_joint_model PRIVATE TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  endif()
endif()

ament_package()
