
# aruco_search

Library that search the ArUco Marker in the room moving the manipulator.

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

```roslaunch aruco_search ArUco_Search.launch```

Include the library into your node:

```#include "aruco_search.h"```

## Version

* **ROS:** Melodic+

## Authors

* **Davide Ferrari**

