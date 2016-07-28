cmake_minimum_required(VERSION 2.8.3)
project(robotis_device)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamixel_sdk
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotis_device
  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(robotis_device
  src/robotis_device/robot.cpp
  src/robotis_device/sensor.cpp
  src/robotis_device/dynamixel.cpp
)

add_dependencies(robotis_device ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robotis_device
  ${catkin_LIBRARIES}
)
