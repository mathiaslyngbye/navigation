# ROS Navigation Stack

A 2D navigation stack that takes in information from odometry, sensor
streams, and a goal pose and outputs safe velocity commands that are sent
to a mobile base.

This fork aims to seperate the global planner from the move base module. This is done with the intention running move base in seperate containers, making it possible to outsource global planning to a cloud service. 

Due to compatibility, this fork assumes Ubuntu 16.04 (Xenial Xerus) and ROS Kinetic. As such, only the kinetic-devel branch is developed upon. 

Most testing will be aimed towards a simulated TurtleBot3 burger model. Setup of TurtleBot3 environment, mapping, and navigation may be found below. 


# TurtleBot3
Step by step setup for the TurtleBot3 navigation stack simulations.

## Installation
The following steps assume Ubuntu 16.04 (Xenial Xerus) as the base operating system.

### Installing ROS
http://wiki.ros.org/kinetic/Installation/Ubuntu

### Installing ROS packages
``` bash
sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
```

### Installing TurtleBot3 packages
``` bash
sudo apt-get install ros-kinetic-turtlebot3 ros-kinetic-turtlebot3-msgs ros-kinetic-turtlebot3-simulations
```

## Setting up gazebo environment

### Launch environment world and robot
Several environments are available. The "_world" environment is an example.
``` bash
export TURTLEBOT3_MODEL=burger
```
``` bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Teleoperation
Once the world is launched, the simulated TurtleBot may be controlled by keyboard.
``` bash
export TURTLEBOT3_MODEL=burger
```
``` bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
```

## Simultaneous localization and mapping (SLAM)

### Launching SLAM with gmapping
``` bash
export TURTLEBOT3_MODEL=burger
```
``` bash
roslaunch turtlebot3_slam turtlebot3_gmapping.launch 
```

### Observing map generation in RViz
``` bash
export TURTLEBOT3_MODEL=burger
```
``` bash
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```

To see the map updates on RViz as you drive the robot around, under Map on the left hand side panel, check to see that Topic is set to "/map".

### Saving generated map
While map generation is still running, run the following command (map name may include path to map, e.g. "/tmp/my_map").
``` bash
rosrun map_server map_saver -f <your map name>
```

## Autonomous Navigation of a Known Map
### Prerequisites
A running gazebo robot/environment and a generated map file of said environment (see previous sections).

### Setup
Export your robot model.
``` bash
export TURTLEBOT3_MODEL=burger
```
Export your generated map file.
``` bash
export TURTLEBOT3_MAP_FILE=/tmp/my_map.yaml
```
Alternately,
``` bash
export TURTLEBOT3_MAP_FILE=`rospack find turtlebot3_navigation`/maps/my_map.yaml
```
Good practice for frequently used maps may look like the option 2 (which obviously requires the maps to be located at the given location).

Launch navigation/RViz.
``` bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

### Localize the TurtleBot3
When starting up, the TurtleBot does not know where it is. To provide him its approximate location on the map, in RViz:
1. Click the "2D Pose Estimate" button
2. Click on the map where the TurtleBot approximately is and drag in the direction the TurtleBot is pointing. 

The laser scan should line up approximately with the walls in the map. If things don't line up well you can repeat the procedure. Additionally, if it is not default behavious, set the fixed frame to "map" in RVIZ's global options to use the "2D Pose Estimate button". 

### Send a navigation goal
With the TurtleBot localized, it can then autonomously plan through the environment.
To send a goal, in RViz:
1. Click the "2D Nav Goal" button
2. Click on the map where you want the TurtleBot to drive and drag in the direction the TurtleBot should be pointing at the end. 

This can fail if the path or goal is blocked.
If you want to stop the robot before it reaches it's goal, send it a goal at it's current location.
