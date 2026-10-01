cmake_minimum_required(VERSION 3.8)

project(terrain_analyzer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
)

add_library(terrain_analyzer_core
  src/terrain_analyzer.cpp
)

target_include_directories(
  terrain_analyzer_core PUBLIC
  include
)

target_link_libraries(
  terrain_analyzer_core
  Eigen3::Eigen
)

add_executable(
  test_terrain_analyzer
  src/test_terrain_analyzer.cpp
)

target_link_libraries(
  test_terrain_analyzer
  terrain_analyzer_core
)

install(
  TARGETS
    terrain_analyzer_core
    test_terrain_analyzer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()