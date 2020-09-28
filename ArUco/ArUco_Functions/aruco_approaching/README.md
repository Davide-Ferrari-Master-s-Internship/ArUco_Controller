
# aruco_approaching

Library that moves the mobile base to the ArUco Marker in order to allow the manipulator reach the marker.
Robot needs to be aligned to the ArUco Marker and ArUco Marker in the field of view of the camera.

## Getting Started

### Dependencies

 * vision_opencv
 ```git clone https://github.com/ros-perception/vision_opencv```
 * usb_cam
  ```git clone https://github.com/ros-drivers/usb_cam```
 
 * aruco_detection
 
 * dynamic_planner
 * prbt_planner
 * robot_bridge
 * useful_functions

## Running the tests

Launch the node:

```roslaunch aruco_approaching ArUco_Approaching.launch```

Include the library into your node:

```#include "aruco_approaching.h"```

## Version

* **ROS:** Melodic+

## Authors

* **Davide Ferrari**

