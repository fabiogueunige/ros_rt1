# Research Track Assignment 2

## Short Description
---------------------
This is the second assignment for the Research Track I course of Robotics Engineering Master Degree in Genoa.
This project involves simulating the movement of a robot in a 3D wordl environment. Users can input a specific position that the robot will attempt to reach. Additionally, users can cancel the goal while the robot is in the process of reaching it. Throughout this simulation, it is possible to view key information about the robot by subscribing to messages or services.

Here an example:
![gif funzionamento]()

## Installing and running
-------------------------
The simulator requires:
* [Ros installation](https://wiki.ros.org/ROS/Installation) follow all the instructions to build the ros-workspace

The package installation:
* Download the git repository inside the ros workaspace/src
in path: `~rosworkspace/src`
```
git clone https://github.com/fabiogueunige/ros_rt1.git
```
* Make all the python file executable
in path: `~rosworkspace/src/PACKAGE/scripts`
```
chmod +x *.py
```
* Inside the ros workspace build the package with `catkin_make`
in path: `~rosworkspace`
```
catkin_make
```
* launch the wordl and all the nodes
```
roslaunch PACKAGE assignment1.launch
```
If you want to launch the nodes separately
```
rosrun PACKAGE nodename.py
```
Instead, to call the services developed to see the velocity of the robot and the target coordinates:
* robot info: `rosservice call /dis_speed`
* target coordinates: `rosservice call /last_target`

### Troubleshooting
After closing all the nodes, some gazebo servoces may be remain opened and so, may create some errors relaunching the environments:
if that insert in to the trminal `killall gzserver`

## Contribution
---------------
Please, do not push changes to this project, but if you want you are free to download it.  
The actual organization is composed by two branches:
* master -> There are all the files necessary for the environment and the Robot operation.
* readRes -> All the resources for the Readme construction.

## Possible improvements


## Pseudocode of action_client.py








