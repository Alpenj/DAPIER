cmake_minimum_required(VERSION 3.16)
project(dapier_task_executor)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

add_library(dapier_task_executor SHARED
  src/task_executor.cpp
)
target_include_directories(dapier_task_executor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS dapier_task_executor
  EXPORT export_dapier_task_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_dapier_task_executor HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(dapier_task_executor)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_task_executor test/test_task_executor.cpp)
  if(TARGET test_task_executor)
    target_link_libraries(test_task_executor dapier_task_executor)
  endif()
endif()

ament_package()
