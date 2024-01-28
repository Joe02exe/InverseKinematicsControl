# Kinematic Control

This package is the Inverse-Kinematic-Control package of the second assignment in robotics.

## Team members
Paul Prünster

Johannes Schneider

Matthias Komar

## Organization of the package
The assignment contains the following files:
- `launch/pick-and-place.launch`: is the launchfile you can start the simulation with (see [manual](#manual))
- `inverse_kinematic_control_node`: The code that interfaces with ROS using moveit APIs
- `run_coppelia.py`: The script that runs coppelia
- `scene_code.py`: The code of the scene within coppelia (we provide this, because the code editor within coppelia is not good)
- `scene.ttt`: the coppelia scene
- `slides.pdf`: presentation slides
- `report.pdf`: our report
- `CmMakeLists.txt, package.xml, setup.py`: package-relevant files


## Manual

First start the ROS-Shell and roscore
```shell
~/ros_home/ros-shell
```
Start ROS
```shell
roscore
```

Open another terminal and the ROS-Shell again and start the launchfile of the package by typing the following:

```shell
roslaunch launch/InverseKinematicsControl.launch
```

This should be it. you should see Rviz. With a delay of about 10 seconds, coppelia will start and the program will be executed. The robot should place the objects in the containers and everything should work just fine.

## Problems and Issues
- Franka the robot always tries his best on picking up the cubes and placing them in the bowls, but it can happen, that there is no way in picking them up/ placing them in the bowls due to obstacles. Franka then skips the cube
- It can also happen that Franka the robot picks up cubes and is not able to hold on to them. It is rather rare, but worth mentioning
- It can happen that Franka is to aggressive, moving the cube in a fancy way and flip the cube away

... We love Franka the robot 🥲
