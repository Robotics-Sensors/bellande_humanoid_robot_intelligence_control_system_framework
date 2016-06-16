cmake_minimum_required(VERSION 2.8.3)
project(robotis_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  std_msgs
  robotis_device
  robotis_controller_msgs
  robotis_framework_common
  cmake_modules
  dynamixel_sdk
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotis_controller
  CATKIN_DEPENDS roscpp roslib sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(robotis_controller
  src/robotis_controller/robotis_controller.cpp
)

target_link_libraries(robotis_controller
  yaml-cpp
  ${catkin_LIBRARIES}
)
