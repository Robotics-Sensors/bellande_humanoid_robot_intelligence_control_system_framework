cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_sdk)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_sdk
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(dynamixel_sdk
  src/${PROJECT_NAME}/packet_handler.cpp
  src/${PROJECT_NAME}/protocol1_packet_handler.cpp
  src/${PROJECT_NAME}/protocol2_packet_handler.cpp
  src/${PROJECT_NAME}/group_sync_read.cpp
  src/${PROJECT_NAME}/group_sync_write.cpp
  src/${PROJECT_NAME}/group_bulk_read.cpp
  src/${PROJECT_NAME}/group_bulk_write.cpp
  src/${PROJECT_NAME}/port_handler.cpp
  src/${PROJECT_NAME}_linux/port_handler_linux.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(dynamixel_sdk
  ${catkin_LIBRARIES}
)

