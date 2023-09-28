################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(humanoid_robot_device)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamixel_sdk
)

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
  LIBRARIES humanoid_robot_device
  CATKIN_DEPENDS 
  roscpp 
  dynamixel_sdk
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(humanoid_robot_device
  src/humanoid_robot_device/robot.cpp
  src/humanoid_robot_device/sensor.cpp
  src/humanoid_robot_device/dynamixel.cpp
)
add_dependencies(humanoid_robot_device ${catkin_EXPORTED_TARGETS})
target_link_libraries(humanoid_robot_device ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS humanoid_robot_device
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY devices
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
