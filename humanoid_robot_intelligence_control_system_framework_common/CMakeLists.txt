cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_intelligence_control_system_framework_common)

if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    robotis_device
  )
  
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp robotis_device
  )
  
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
  
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
  
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
  
else()
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(robotis_device REQUIRED)
  
  include_directories(
    include
  )
  
  add_library(${PROJECT_NAME} INTERFACE)
  
  target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  
  ament_target_dependencies(${PROJECT_NAME} INTERFACE
    rclcpp
    robotis_device
  )
  
  install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  
  install(DIRECTORY include/
    DESTINATION include
  )
  
  ament_export_include_directories(include)
  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  ament_export_dependencies(
    rclcpp
    robotis_device
  )
  
  ament_package()
endif()
