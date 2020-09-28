cmake_minimum_required(VERSION 2.8.3)
project(dynamic_planner)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_planner
  #interactivity_utils
  INCLUDE_DIRS
  ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
  DEPENDS
  EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(dynamic_planner src/dynamic_planner.cpp)
target_link_libraries(dynamic_planner ${catkin_LIBRARIES})
add_dependencies(dynamic_planner ${catkin_EXPORTED_TARGETS})

add_executable(dynamic_planner_node src/dynamic_planner.cpp src/dynamic_planner_node.cpp)
target_link_libraries(dynamic_planner_node ${catkin_LIBRARIES})
add_dependencies(dynamic_planner_node ${catkin_EXPORTED_TARGETS})

