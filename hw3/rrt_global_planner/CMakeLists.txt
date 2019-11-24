cmake_minimum_required(VERSION 2.8.3)
project(rrt_global_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

include(cmake/FindOMPL.cmake)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  roscpp
  rospy
  std_msgs
  tf
)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrt_global_planner
#  CATKIN_DEPENDS base_local_planner costmap_2d geometry_msgs nav_core roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

link_libraries(${OMPL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_lib src/rrt_global_planner.cpp)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES}
        ${OMPL_LIBRARIES})
