
# aruco_controller

Node that controls the robot in order to achieve the picking experiment with ArUco Marker.
Using ArUco_Functions Library the robot does the following actions:

	* Search ArUco Marker in the room
	* Alingment with ArUco Marker
	* Approach ArUco Marker moving Mobile Base
	* Picking Experiment using the Manipulator

## Getting Started

### Dependencies

 * vision_opencv
 ```git clone https://github.com/ros-perception/vision_opencv```
 * usb_cam
  ```git clone https://github.com/ros-drivers/usb_cam```
 
 * aruco_detection
 * aruco_search
 * aruco_alignment
 * aruco_approaching
 * aruco_picking
 
 * dynamic_planner
 * prbt_planner
 * robot_bridge

## Running the tests

Launch the node:

```roslaunch aruco_controller ArUco_Controller.launch```

Launch without real camera:

```roslaunch aruco_controller ArUco_Controller.launch real_camera:=false```

Launch without aruco detection:

```roslaunch aruco_controller ArUco_Controller.launch aruco_detection:=false```

## Version

* **ROS:** Melodic+

## Authors

* **Davide Ferrari**

