cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(realsense_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##########################
# Find Dependencies
##########################
set(
  THIS_PACKAGE_DEPS
    ament_cmake
    rclcpp
    rclcpp_components
#    ros-humble-realsense2-camera
#    ros-humble-realsense2-camera-msgs
#    ros-humble-realsense2-description
    ament_index_cpp
    sensor_msgs
    pcl_conversions
    pcl_ros
  )

foreach(PACKAGE ${THIS_PACKAGE_DEPS})
  find_package(${PACKAGE} REQUIRED)
endforeach()

find_package(PCL REQUIRED COMPONENTS common filters)

##########################
# Build VoxelGrid Component
##########################
add_library(voxel_grid_component SHARED
  src/voxel_grid_node.cpp
)

target_include_directories(voxel_grid_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(voxel_grid_component
  ${PCL_LIBRARIES}
)

ament_target_dependencies(voxel_grid_component
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
  pcl_ros
)

rclcpp_components_register_node(voxel_grid_component
  PLUGIN "realsense_launch::VoxelGridNode"
  EXECUTABLE voxel_grid_node
)

##########################
# Installation
##########################

install(TARGETS
  voxel_grid_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    bags
    config
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
