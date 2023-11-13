cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_intelligence_control_system_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    sensor_msgs
    humanoid_robot_intelligence_control_system_controller_msgs
    dynamixel_sdk
    humanoid_robot_intelligence_control_system_device
    humanoid_robot_intelligence_control_system_framework_common
    cmake_modules
  )

  find_package(Boost REQUIRED)

  find_package(PkgConfig REQUIRED)
else()
  find_package(ament_cmake REQUIRED)
endif()


pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")


if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES humanoid_robot_intelligence_control_system_controller
    CATKIN_DEPENDS 
    roscpp 
    roslib 
    std_msgs 
    sensor_msgs 
    humanoid_robot_intelligence_control_system_controller_msgs 
    dynamixel_sdk 
    humanoid_robot_intelligence_control_system_device 
    humanoid_robot_intelligence_control_system_framework_common 
    cmake_modules
    DEPENDS Boost
  )
endif()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(humanoid_robot_intelligence_control_system_controller src/humanoid_robot_intelligence_control_system_controller/humanoid_robot_intelligence_control_system_controller.cpp)
add_dependencies(humanoid_robot_intelligence_control_system_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(humanoid_robot_intelligence_control_system_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES})

install(TARGETS humanoid_robot_intelligence_control_system_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
