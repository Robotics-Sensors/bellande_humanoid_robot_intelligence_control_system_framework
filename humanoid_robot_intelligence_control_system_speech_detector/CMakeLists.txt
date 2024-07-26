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

cmake_minimum_required(VERSION 3.0.2)
project(humanoid_robot_intelligence_control_system_speech_detector)

if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    cv_bridge
  )

  catkin_package(
    CATKIN_DEPENDS
      rospy
      std_msgs
      sensor_msgs
      geometry_msgs
      cv_bridge
  )

else()
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rclpy REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
endif()

if($ENV{ROS_VERSION} EQUAL 1)
  catkin_python_setup()

  catkin_install_python(PROGRAMS
    src/speech_detection_processor.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY config launch rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

else()
  ament_python_install_package(${PROJECT_NAME})

  install(PROGRAMS
    src/speech_detection_processor.py
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY config launch rviz
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()
endif()
