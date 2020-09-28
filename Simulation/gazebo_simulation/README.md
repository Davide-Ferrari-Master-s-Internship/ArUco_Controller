
# gazebo_simulation

Package that contains the gazebo simulations of the rossini project in the IMA environment, using the robots Neobotix MPO_500 and Pilz PRBT.

### Dependencies

 * pilz_robots 
 ```(git clone https://github.com/PilzDE/pilz_robots```
 * timed_roslaunch
 ```git clone https://github.com/MoriKen254/timed_roslaunch.git```
 * robot_bridge
 * gazebo9

## Running the tests

Launch simulation in Empty_World:

```roslaunch gazebo_simulation rossini_gazebo.launch```

Launch simulation in ArUco_World:

```roslaunch gazebo_simulation rossini_gazebo.launch empty_world:=false aruco_world:=true```

Launch simulation in IMA_World:

```roslaunch gazebo_simulation rossini_gazebo.launch empty_world:=false IMA_world:=true```

Launch simulation with real camera (default is virtual camera):

```roslaunch gazebo_simulation rossini_gazebo.launch real_camera:=true```

## Version

* **ROS:** Melodic+

## Authors

* **Davide Ferrari**
