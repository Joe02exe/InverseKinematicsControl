# Kinematic Control

This package is the Inverse-Kinematic-Control package of the second assignment in robotics.

## Team members
Paul Prünster

Johannes Schneier

Matthias Komar

## Manual


First start the ROS-Shell and roscore
```shell
~/ros_home/ros-shell
```
Start ROS
```shell
roscore
```

Open another terminal and the ROS-Shell and start the launchfile of the package by typing the following:

```shell
roslaunch launch/InverseKinematicsControl.launch
```

This should be it. you should see Rviz. With a delay of about 10 seconds, coppelia will start and the program will be executed. The robot should place the objects in the containers and everything should work just fine.


### Safe this for later maybe
obstacle:
dimensions for Object_3: [0.05000000447034836, 0.05000000447034836, 0.3499999940395355, 0.0]

container
dimensions for Object_3: [0.11835280805826187, 0.11835280805826187, 0.010370612144470215, 0.0]

objects:
dimensions for Object_1: [0.05000000074505806, 0.05000000074505806, 0.05000000074505806, 0.0]
