# Research Track Assignment 2
================================

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
```
path: ~rosworkspace/src
git clone https://github.com/fabiogueunige/ros_rt1.git
```
* Make all the python file executable
```
path: ~rosworkspace/src/PACKAGE/scripts
chmod +x *.py
```
* Inside the ros workspace build the package with `catkin_make`
```
path: ~rosworkspace
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


## Assignment 2 (to do)
Problem here! The task is blocking and I cannot do anything while the robot is reaching the target.
This is one of the long-running tasks that should be implemented with an action server!
[The package assignment_2_2022](https://github.com/CarmineD8/assignment_2_2023)
provides an implementation of the same node as an action server
What should you do here?
- Create a new package, in which you will develop three nodes:
- (a) A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Try to use the
feedback/status of the action server to know when the target has been reached. The node also publishes the
robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the
topic /odom;
- (b) A service node that, when called, returns the coordinates of the last target sent by the user;
- (c) Another service node that subscribes to the robot’s position and velocity (using the custom message) and
implements a server to retrieve the distance of the robot from the target and the robot’s average speed.
- Create a launch file to start the whole simulation. Use a parameter to select the size of the averaging window of node (c)

### Additional Requirements:
- Only for node (a): Create a flowchart of your code, or describe it in pseudocode (Pseudocode Examples
(unf.edu))
- Add some comments to the code
- Use functions to avoid having a single block of code
- Publish the new package on your own repository. The flowchart (or the pseudocode) should be added to the
ReadMe of the repository. (consider using Markdown syntax to write your readme: Basic Syntax | Markdown
Guide)

### Deadline
Deadline: 12/01/2024

### Evaluetion
- Code performance
- Code structure and clarity
- Respect of the requirements
- Organization of the repository (e.g., README in which you describe what the code does (possibly with
flowchart or pseudocode), how to run the code, possible improvements, … )






