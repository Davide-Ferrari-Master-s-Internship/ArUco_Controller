# robot_bridge

Simple explanation of your package

## Getting Started

Node that interface directly with robot.

There is an upper level node that split a custom trajectory message (manipulator + mobile base) in a trajectory message for maipulator and a velocity message for mobile base.

The other two nodes use thoose message to control directly the manipulator and the mobile base.

## Running the tests

Run the node

```roslaunch robot_bridge Bridge.launch```

## Version

* **ROS:** Melodic+

## Authors

* **Davide Ferrari**
