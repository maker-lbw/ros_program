cmake_minimum_required(VERSION 3.0.2)
project(simple_local_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simple_local_planner
  CATKIN_DEPENDS costmap_2d geometry_msgs nav_core nav_msgs pluginlib roscpp tf2 tf2_geometry_msgs tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(simple_local_planner
  src/simple_local_planner.cpp
)

target_link_libraries(simple_local_planner
  ${catkin_LIBRARIES}
)

install(TARGETS simple_local_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
  simple_local_planner.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
