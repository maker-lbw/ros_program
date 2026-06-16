cmake_minimum_required(VERSION 3.0.2)
project(smooth_waypoint_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  std_srvs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES smooth_waypoint_planner
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    std_srvs
    tf2
    tf2_geometry_msgs
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(smooth_waypoint_planner SHARED
  src/catmull_rom.cpp
  src/path_processing.cpp
  src/smooth_waypoint_planner.cpp
)

target_link_libraries(smooth_waypoint_planner
  ${catkin_LIBRARIES}
)

install(TARGETS smooth_waypoint_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_path_processing
    test/test_path_processing.cpp
    src/path_processing.cpp
  )

  if(TARGET test_path_processing)
    target_link_libraries(test_path_processing
      ${catkin_LIBRARIES}
    )
  endif()
endif()
