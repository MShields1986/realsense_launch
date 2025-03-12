cmake_minimum_required(VERSION 3.8)
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
  )

foreach(PACKAGE ${THIS_PACKAGE_DEPS})
  find_package(${PACKAGE} REQUIRED)
endforeach()

##########################
# Installation
##########################

install(
  DIRECTORY
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
