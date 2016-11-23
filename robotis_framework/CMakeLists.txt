cmake_minimum_required(VERSION 2.8.3)
project(robotis_framework)
find_package(catkin REQUIRED)
catkin_metapackage()

Invalid metapackage:
  Metapackage 'robotis_framework': Invalid CMakeLists.txt
Expected:
<<<cmake_minimum_required(VERSION 2.8.3)
project(robotis_framework)
find_package(catkin REQUIRED)
catkin_metapackage()
>>>
Got:
<<<cmake_minimum_required(VERSION 2.8.3)
project(robotis_framework)
find_package(catkin REQUIRED)
catkin_metapackage()
>>>
