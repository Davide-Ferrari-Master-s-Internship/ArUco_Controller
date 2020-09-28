cmake_minimum_required(VERSION 2.8.3)
project(aruco_alignment)

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
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco_alignment
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

##################################################################################################

# ArUco Alignment Library

add_library(aruco_alignment src/aruco_alignment.cpp)
target_link_libraries(aruco_alignment ${catkin_LIBRARIES})
add_dependencies(aruco_alignment ${catkin_EXPORTED_TARGETS})


add_executable(aruco_alignment_Node src/aruco_alignment_Node.cpp src/aruco_alignment.cpp)
target_link_libraries(aruco_alignment_Node ${catkin_LIBRARIES})
add_dependencies(aruco_alignment_Node ${catkin_EXPORTED_TARGETS})


##################################################################################################
