cmake_minimum_required(VERSION 3.0.2)
project(car_driver)

find_package(catkin REQUIRED COMPONENTS
		message_generation
		message_runtime
		roscpp
		roslaunch
		)

add_message_files(FILES
    RealCMD.msg
		)


generate_messages()

catkin_package(CATKIN_DEPENDS message_generation message_runtime roscpp)

include_directories(include  ${catkin_INCLUDE_DIRS})

link_directories(
  lib
  ${catkin_LIB_DIRS}
)

add_executable(car_hardware_node 
  node/car_hardware_node.cpp 
  src/wheel_motor_driver.cpp
  src/car_canalyst.cpp
)
add_dependencies(car_hardware_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(car_hardware_node
  ${catkin_LIBRARIES}
  controlcan
)
