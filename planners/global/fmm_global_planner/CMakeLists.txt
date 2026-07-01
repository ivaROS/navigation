cmake_minimum_required(VERSION 3.0.2)
project(fmm_global_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_core
  costmap_2d
  geometry_msgs
  nav_msgs
  pluginlib
  message_generation
)

add_service_files(
  FILES
  FMMPlan.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES fmm_global_planner
  CATKIN_DEPENDS
    roscpp
    rospy
    nav_core
    costmap_2d
    geometry_msgs
    nav_msgs
    pluginlib
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(fmm_global_planner
  src/fmm_global_planner.cpp
)

add_dependencies(fmm_global_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(fmm_global_planner
  ${catkin_LIBRARIES}
)

install(TARGETS fmm_global_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES global_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/fmm_plan_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)