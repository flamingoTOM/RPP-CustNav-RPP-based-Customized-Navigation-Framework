cmake_minimum_required(VERSION 3.0.2)
project(rpp_pkg)

find_package(catkin REQUIRED COMPONENTS
  angles
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  roscpp
  tf2_geometry_msgs
  tf2_ros
)

# 查找 glog 库
find_package(glog REQUIRED)

# 查找 Python 和 NumPy 库 (更新为现代 CMake 风格)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

catkin_package(
  INCLUDE_DIRS include  # 导出 include 目录
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${catkin_PACKAGES}
)

# 找到 include_directories 部分，添加下面这一行
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${NUMPY_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/matplotlib-cpp  # 新增：指向存放 matplotlibcpp.h 的目录
)

# 定义库目标并链接
add_library(${PROJECT_NAME}
  src/controller.cpp
  src/rpp_controller.cpp
  src/rpp_math.cpp  
  src/rpp_vec2d.cpp
  src/curve.cpp
  src/bspline_curve.cpp 
  src/point_global_planner.cpp  # 新增
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  glog::glog
  ${Python3_LIBRARIES}
)

# 添加 Python3 NumPy 依赖
target_include_directories(${PROJECT_NAME} PUBLIC
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)

# 定义可执行文件目标
add_executable(path_optimizer src/path_optimizer.cpp)
add_executable(path_visualization src/path_visualization.cpp)
add_executable(douglas src/douglas.cpp)
add_executable(nav_goal_publish src/nav_goal_publish.cpp)

add_dependencies(path_optimizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(path_visualization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(douglas ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(nav_goal_publish ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 链接可执行文件
target_link_libraries(path_optimizer
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  glog::glog
  ${Python3_LIBRARIES}
)

target_link_libraries(path_visualization
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  glog::glog
  ${Python3_LIBRARIES}
)

target_link_libraries(douglas
  ${catkin_LIBRARIES}
)

target_link_libraries(nav_goal_publish
  ${catkin_LIBRARIES}
)

add_compile_options(-finput-charset=UTF-8 -fexec-charset=UTF-8)
add_compile_options(-DPYTHON_VERSION=3)  # 明确指定 Python 版本

# 安装规则
install(TARGETS path_visualization path_optimizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)