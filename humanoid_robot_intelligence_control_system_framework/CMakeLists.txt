cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_intelligence_control_system_framework)

if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED)
else()
  find_package(ament_cmake REQUIRED)
endif()

if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package()
endif()
