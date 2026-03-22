cmake_minimum_required(VERSION 3.0.2)
project(car_move)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslaunch
  tf2_ros
  dynamic_reconfigure
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES car_move
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../devel/include
  ${catkin_INCLUDE_DIRS}

)

add_executable(control_main_node 
          node/control_main_node.cpp
          src/car_control.cpp)

target_link_libraries(control_main_node
${catkin_LIBRARIES}
)


