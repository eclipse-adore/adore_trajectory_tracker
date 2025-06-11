cmake_minimum_required(VERSION 3.8)
project(trajectory_tracker LANGUAGES CXX)

# C++20 strict mode
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build config
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(adore_math REQUIRED)
find_package(adore_controllers REQUIRED)
find_package(adore_dynamics REQUIRED)
find_package(adore_dynamics_conversions REQUIRED)
find_package(adore_map REQUIRED)
find_package(adore_ros2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

# Executable
add_executable(${PROJECT_NAME} src/trajectory_tracker_node.cpp)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  adore_math
  adore_dynamics
  adore_dynamics_conversions
  adore_map
  adore_controllers
  adore_ros2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  osqp
  OsqpEigen
)

# Component library
add_library(${PROJECT_NAME}_component SHARED src/trajectory_tracker_node.cpp)

target_include_directories(${PROJECT_NAME}_component
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  adore_math
  adore_dynamics
  adore_dynamics_conversions
  adore_map
  adore_controllers
  adore_ros2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  osqp
  OsqpEigen
)

# Register the component
rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "adore::TrajectoryTrackerNode"
  EXECUTABLE trajectory_tracker_exec
)

# Installation
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Headers
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
