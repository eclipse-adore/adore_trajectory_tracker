cmake_minimum_required(VERSION 3.8)
project(trajectory_tracker)


include(../../../libraries/adore_libraries.cmake)

include_directories(include)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
# Ensure the Release build uses -O3 for optimization
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(trajectory_tracker_node src/trajectory_tracker_node.cpp)

include(requirements.cmake)

target_include_directories(trajectory_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_all_target_include_directories(trajectory_tracker_node)
add_all_target_link_libraries(trajectory_tracker_node)

#ament_target_dependencies(vehicle_model_node rclcpp adore_ros2_msgs)

install(TARGETS
  trajectory_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
