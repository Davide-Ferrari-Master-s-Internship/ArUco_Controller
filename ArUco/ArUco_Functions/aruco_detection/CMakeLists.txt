cmake_minimum_required(VERSION 2.8.3)
project(aruco_detection)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  OpenCV
)

catkin_package(
  INCLUDE_DIRS include
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ArUco_Detection_Node src/ArUco_Detection_Node.cpp src/ArUco_Detection.cpp)
target_link_libraries(ArUco_Detection_Node ${catkin_LIBRARIES})
add_dependencies(ArUco_Detection_Node ${catkin_EXPORTED_TARGETS})


