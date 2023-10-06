cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_device)


if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    dynamixel_sdk
  )
else()
  find_package(ament_cmake REQUIRED)
endif()


if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES humanoid_robot_device
    CATKIN_DEPENDS 
    roscpp 
    dynamixel_sdk
  )
endif()


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
