# Copyright (C) 2025 Bellande Robotics Sensors Research Innovation Center, Ronaldson Bellande
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

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES humanoid_robot_intelligence_control_system_device
    CATKIN_DEPENDS
    roscpp
    dynamixel_sdk
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

else()
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(dynamixel_sdk REQUIRED)

  include_directories(
    include
  )
endif()

add_library(humanoid_robot_intelligence_control_system_device
  src/humanoid_robot_intelligence_control_system_device/robot.cpp
  src/humanoid_robot_intelligence_control_system_device/sensor.cpp
  src/humanoid_robot_intelligence_control_system_device/dynamixel.cpp
)

if($ENV{ROS_VERSION} EQUAL 1)
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

else()
  ament_target_dependencies(humanoid_robot_intelligence_control_system_device
    rclcpp
    dynamixel_sdk
  )

  install(TARGETS humanoid_robot_intelligence_control_system_device
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(DIRECTORY include/
    DESTINATION include
  )

  install(DIRECTORY devices
    DESTINATION share/${PROJECT_NAME}
  )

  ament_export_include_directories(include)
  ament_export_libraries(humanoid_robot_intelligence_control_system_device)
  ament_export_dependencies(rclcpp dynamixel_sdk)

  ament_package()
endif()
