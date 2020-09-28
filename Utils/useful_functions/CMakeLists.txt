cmake_minimum_required(VERSION 2.8.3)
project(useful_functions)

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
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES useful_functions
  DEPENDS EIGEN3
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(useful_functions src/useful_functions.cpp)
target_link_libraries(useful_functions ${catkin_LIBRARIES})
add_dependencies(useful_functions ${catkin_EXPORTED_TARGETS})

