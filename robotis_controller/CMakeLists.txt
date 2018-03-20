################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(robotis_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  robotis_controller_msgs
  dynamixel_sdk
  robotis_device
  robotis_framework_common
  cmake_modules
)

find_package(yaml-cpp REQUIRED)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
# pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotis_controller
  CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs robotis_controller_msgs dynamixel_sdk robotis_device robotis_framework_common cmake_modules
  DEPENDS YAML_CPP
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIRS}
)

add_library(robotis_controller src/robotis_controller/robotis_controller.cpp)
add_dependencies(robotis_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(robotis_controller ${yaml_cpp_LIBRARIES} ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS robotis_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
