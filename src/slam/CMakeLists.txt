cmake_minimum_required(VERSION 3.0.2)
project(xj_robot_slam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
  std_msgs
  tf
  tf_conversions
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xj_robot_slam
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
add_executable(calc_odom src/calc_odom.cpp)
add_dependencies(calc_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(calc_odom ${catkin_LIBRARIES})