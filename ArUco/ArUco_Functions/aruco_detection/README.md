
# aruco_detection

Node that detect and compute the relative position of an ArUco Marker in respect to a webcam.
The coordinates and the identifier of the ArUco are printed on the scene recorded by the camera and published on topics.

## Getting Started

### Dependencies

 * vision_opencv
 ```git clone https://github.com/ros-perception/vision_opencv```
 * usb_cam
  ```git clone https://github.com/ros-drivers/usb_cam```
 
## Running the tests

Launch the node:

```roslaunch aruco_detection Aruco_Detection.launch```

## Version

* **ROS:** Melodic+

## Authors

* **Davide Ferrari**

