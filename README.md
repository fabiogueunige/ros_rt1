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
* Xterm installation. Run:
`$ sudo apt-get -y install xterm`
The package installation:
* Download the git repository inside the ros workaspace/src
in path: `~rosworkspace/src`
```
$ git clone https://github.com/fabiogueunige/ros_rt1.git
```
* Make all the python file executable
in path: `~rosworkspace/src/PACKAGE/scripts`
```
$ chmod +x *.py
```
* Inside the ros workspace build the package with `catkin_make`
in path: `~rosworkspace`
```
$ catkin_make
```
* launch the wordl and all the nodes
```
$ roslaunch assignment_2_2023 assignment1.launch
```
If you want to launch the nodes separately
```
$ rosrun assignment_2_2023 nodename.py
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
------------------------
This project has several areas of improvements:
* movement: The Robot moves really slow and the movement decision is not the optimal one. In fact it would be possible to make the robot rotate in the best possible direction and not always in the prefixed one. It would be possible also to make the robot faster especially when it detects the wall and has to rotate.
* robot message: The message and the service showed often has some accuracy errors. So it may be useful to add some threshold to hide them.
* User Interface: The usage of Xterm library improves a lot the user interface, but it is still minimalist and difficult to understand. So it may be a good implementation add more user input directly from more than one terminal (e.g to run all the services developed).
* environment: The environment for the robot movement can be updated adding some obstacles or a graphical view of the desired goal directly into the interface.

## Code explenation
-------------------
This directory is a follow up of [link](https://github.com/CarmineD8/assignment_2_2023.git)
So here a fast introduction of the nodes already implemented:
**bug_as.py:**Decides the robot operation in base of its state developing the information given by the services and the messages such as the laser scan and the odometry
**go_to_point_service.py:** Implements a finite sate machine that check and update the status of the robot depending on reaching the desired position.
**wall_follow_service:** Makes the robot follow a wall to drive around it. In the meanwhile it processes datas to detect obstacles and avoid them with an 180 degrees view (front, left and right).

### Assignment implementation


### Pseudocode actioncllient.py








