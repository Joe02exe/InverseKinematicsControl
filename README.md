# Kinematic Control
## Team member
Matthias Ebner

Stefan Huter

Paul Prünster

## Manual

Start the virtual machine and ROS
```shell
~/ros_home/ros-shell
```
Start ROS
```shell
roscore
```
Open a other terminal and VM to start coppeliaSim
```shell
coppeliaSim.sh ./coppelia-scene.ttt
```

Then start a VM an make sure you are in the kinematic_control directory to launch kinematic control
```shell
cd ~/catkin_ws/src/kinematic_control/
```
```shell
roslaunch launch/kinematic_control.launch
```

To run the simulation open a other VM and enter one of the following publishes
```shell
rostopic pub -l /Franka/goal_euler kinematic_control/EulerPose "{position: {x: 0.366, y: -0.15, z: 0.405}, orientation: {phi: 3.1415, theta: 3.1415, psi: 0.0}}"

rostopic pub -l /Franka/goal_euler kinematic_control/EulerPose "{position: {x: 0.366, y: -0.15, z: 0.405}, orientation: {phi: 3.3, theta: 2.8, psi: 0.0}}"

rostopic pub -l /Franka/goal_euler kinematic_control/EulerPose "{position: {x: 0.366, y: -0.15, z: 0.205}, orientation: {phi: 3.3, theta: 2.8, psi: 0.0}}"

rostopic pub -l /Franka/goal_euler kinematic_control/EulerPose "{position: {x: 0.366, y: -0.15, z: 0.405}, orientation: {phi: 0.0, theta: 3.1415, psi: 4.0}}"
```