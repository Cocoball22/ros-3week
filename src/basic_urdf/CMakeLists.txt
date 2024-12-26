cmake_minimum_required(VERSION 3.0.2)
project(basic_urdf)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  sensor_msgs 
  OpenCV
  # cmath
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES basic_urdf
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

#add_library(${PROJECT_NAME}
# src/${PROJECT_NAME}/main_scan_depth.cpp
#)

add_executable(scan_depth src/main_scan_depth.cpp
                          src/scan_depth.cpp)
 
 
 add_dependencies(scan_depth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 
 target_link_libraries(scan_depth
    ${catkin_LIBRARIES}
)


