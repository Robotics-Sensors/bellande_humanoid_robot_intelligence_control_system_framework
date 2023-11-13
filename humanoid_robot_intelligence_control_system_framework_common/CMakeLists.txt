cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_intelligence_control_system_framework_common)


if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    humanoid_robot_intelligence_control_system_device
  )
else()
  find_package(ament_cmake REQUIRED)
endif()


if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
    roscpp 
    humanoid_robot_intelligence_control_system_device
  )
endif()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/motion_module.h
  include/${PROJECT_NAME}/sensor_module.h
  include/${PROJECT_NAME}/singleton.h
)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

install(TARGETS humanoid_robot_intelligence_control_system_framework_common
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
