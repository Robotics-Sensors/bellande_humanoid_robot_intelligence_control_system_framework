# Copyright (C) 2024 Bellande Robotics Sensors Research Innovation Center, Ronaldson Bellande
# 
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
# 
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_intelligence_control_system_device)


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
    LIBRARIES humanoid_robot_intelligence_control_system_device
    CATKIN_DEPENDS 
    roscpp 
    dynamixel_sdk
  )
endif()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(humanoid_robot_intelligence_control_system_device
  src/humanoid_robot_intelligence_control_system_device/robot.cpp
  src/humanoid_robot_intelligence_control_system_device/sensor.cpp
  src/humanoid_robot_intelligence_control_system_device/dynamixel.cpp
)
add_dependencies(humanoid_robot_intelligence_control_system_device ${catkin_EXPORTED_TARGETS})
target_link_libraries(humanoid_robot_intelligence_control_system_device ${catkin_LIBRARIES})

install(TARGETS humanoid_robot_intelligence_control_system_device
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
