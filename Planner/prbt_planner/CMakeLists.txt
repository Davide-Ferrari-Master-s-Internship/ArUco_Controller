cmake_minimum_required(VERSION 2.8.3)
project(prbt_planner)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS

  roscpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  
  message_generation
  
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
  
  robot_bridge
  dynamic_planner
)

find_package(Boost REQUIRED system filesystem date_time thread)

add_service_files(
   FILES
   PlanJointSpace.srv
   PlanCartesianSpace.srv
   SetPlannerParameters.srv
   SetPlannerDynamics.srv
   ComputeEndEffectorPosition.srv
   GetManipulatorLinkNames.srv
   GetManipulatorInfo.srv
   GetManipulatorTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES prbt_planner
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

link_directories(${catkin_LIBRARY_DIRS})

##################################################################################################

# ArUco Search Library

add_library(prbt_planner src/prbt_planner.cpp)
target_link_libraries(prbt_planner ${catkin_LIBRARIES})
add_dependencies(prbt_planner ${catkin_EXPORTED_TARGETS})

add_executable(prbt_planner_Node src/prbt_planner_Node.cpp src/prbt_planner.cpp)
target_link_libraries(prbt_planner_Node ${catkin_LIBRARIES})
add_dependencies(prbt_planner_Node ${catkin_EXPORTED_TARGETS})

##################################################################################################
