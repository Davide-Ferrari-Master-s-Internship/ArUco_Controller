
# aruco_alignment

Library that align the robot (Mobile Base and Manipulator) with ArUco Marker.
ArUco Marker needs to be in the field of view of camera.

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

```roslaunch aruco_alignment ArUco_Alignment.launch```

Include the library into your node:

```#include "aruco_alignment.h"```

## Version

* **ROS:** Melodic+

## Authors

* **Davide Ferrari**

