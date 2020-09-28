cmake_minimum_required(VERSION 2.8.3)
project(aruco_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  
  robot_bridge
  
  dynamic_planner
  prbt_planner
  useful_functions
  
  aruco_search
  aruco_alignment
  aruco_approaching
  aruco_picking
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(aruco_controller_Node src/aruco_controller_Node.cpp src/aruco_controller.cpp)
target_link_libraries(aruco_controller_Node ${catkin_LIBRARIES})
add_dependencies(aruco_controller_Node ${catkin_EXPORTED_TARGETS})

