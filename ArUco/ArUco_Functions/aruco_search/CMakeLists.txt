cmake_minimum_required(VERSION 2.8.3)
project(aruco_search)

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
  LIBRARIES aruco_search
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

##################################################################################################

# ArUco Search Library

add_library(aruco_search src/aruco_search.cpp)
target_link_libraries(aruco_search ${catkin_LIBRARIES})
add_dependencies(aruco_search ${catkin_EXPORTED_TARGETS})

add_executable(aruco_search_Node src/aruco_search_Node.cpp src/aruco_search.cpp)
target_link_libraries(aruco_search_Node ${catkin_LIBRARIES})
add_dependencies(aruco_search_Node ${catkin_EXPORTED_TARGETS})


##################################################################################################
