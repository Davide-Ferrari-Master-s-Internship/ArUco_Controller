cmake_minimum_required(VERSION 2.8.3)
project(aruco_approaching)

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
  LIBRARIES aruco_approaching
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

##################################################################################################

# ArUco Approaching Library

add_library(aruco_approaching src/aruco_approaching.cpp)
target_link_libraries(aruco_approaching ${catkin_LIBRARIES})
add_dependencies(aruco_approaching ${catkin_EXPORTED_TARGETS})

add_executable(aruco_approaching_Node src/aruco_approaching_Node.cpp src/aruco_approaching.cpp)
target_link_libraries(aruco_approaching_Node ${catkin_LIBRARIES})
add_dependencies(aruco_approaching_Node ${catkin_EXPORTED_TARGETS})

##################################################################################################
