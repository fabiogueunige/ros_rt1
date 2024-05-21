# Research Track Assignment 2

## Short Description
---------------------
This is the second assignment for the Research Track I course of Robotics Engineering Master Degree in Genoa.
This project involves simulating the movement of a robot in a 3D wordl environment. Users can input a specific position that the robot will attempt to reach. Additionally, users can cancel the goal while the robot is in the process of reaching it. Throughout this simulation, it is possible to view key information about the robot by subscribing to messages or services.

Here an example: 

![gif funzionamento](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExdG9ydG1zbzU2MnUxdmFkdGt5ZjFheWk4ZzFlZ3dqd2hxeHd3MWd1OSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/wPymwQwoxHJQOBmrak/giphy-downsized-large.gif)

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

### Documentation
[ros_rt1 documentation](https://fabiogueunige.github.io/ros_rt1/)

### Introduction
This directory is a follow up of [CarmineD8/assignment_2_2023
](https://github.com/CarmineD8/assignment_2_2023.git).  
So here a fast introduction of the nodes already implemented:
**bug_as.py:** Decides the robot operation in base of its state developing the information given by the services and the messages such as the laser scan and the odometry.  
**go_to_point_service.py:** Implements a finite sate machine that check and update the status of the robot depending on reaching the desired position. 
 
 **wall_follow_service:** Makes the robot follow a wall to drive around it. In the meanwhile it processes datas to detect obstacles and avoid them with an 180 degrees view (front, left and right).

### Assignment implementation
The assignment requires the implementation of the desired position acquisition user side.
The nodes implemented are:
* **action_client.py** to send to bug_as.py the goal position chose by the user and to publish all the information about the actual position of the robot and its velocity. The message is the RobotInfo.msg inside the msg folder.
*  **last_tar_service.py** to create a service that can be call by the user with the position of the target informations. The service is LastTarget.srv inside the srv folder
*  **dis_speed_service** to retrieve the distance of the robot from the target and the avarage speed of the robot along the x axis and the angular speed in case of rotation.  
[Here the documentation page](https://fabiogueunige.github.io/ros_rt1/)

![Scheme show](https://github.com/fabiogueunige/ros_rt1/blob/readRes/rosgraph.png)

### Pseudocode actioncllient.py
Interpreter declaration for Python3
`#! /usr/bin/env python3`

Import of all the necessary libraries

Declaration of the global variables needed by more functions
```
- pubInfo # publisher of RobotInfo message
- subOdom # Subscriber to Odom topic
```
insertNumber() definition:
  function to take the user input for the target
```
  while()
    inp = user input
    try: # to stay alive also with some errors
      inp = float transformation from string
      break to exit from the while
    except:
       print() # advice the user to insert numbers
# This is develop for x input and y input 
```
odom_callback(message) definition:
  function called when the subscriber receives the information from the Odom topic
```
  global pubInfo # to add the publisher to the method

  msgInfo declaration as RobotInfo object
  msgInfo = Robotinfo()
  msgInfo.pos_x = position of x of the Robot
  msgInfo.pos_y = position as y of the Robot
  msgInfo.vel_x = velocity along x of the Robot
  msgInfo.vel_ang_z = velocity along y of the Robot
 
  pubInfo.publish(msgInfo) # Publish the message
```
action_client() definition:
  function to implement the client to reaching goal topic to see the state of the Robot in relationship with the target of the user
```
  global clienttar

  clienttar.wait_for_server() # Wait until the action server is up and running.
  
  # User interface initialization
  print("")
  print("")
  print("")

  goal = Definition of the goal to send to the action server
 
  inpx, inpy = insertNumber() # coordinates acquisition
  
  # Update the goal
  goal.target_pose.pose.position.x = inpx
  goal.target_pose.pose.position.y = inpy
  
  
  clienttar.send_goal(goal) # Sending the goal
  print("") # To user comunication
  
  while not rospy.is_shutdown(): # while until the node is alive
  # New inputs from the user to decide what to do
  choice = input(if to insert a new goal)
  if (choice == 'y'):
      inpx, inpy = insertNumber() # coordinates acquisition
  
      # Update the goal
      goal.target_pose.pose.position.x = inpx
      goal.target_pose.pose.position.y = inpy
  
      
      clienttar.send_goal(goal) # Send the goal
      print() # To user comunication
  else:
      choice = input("if to cancel the goal")
      if (choice == 'y'):
          if (the goal is in progress):
              print(goal information)
              clienttar.cancel_goal() # cancel the goal
          else:
              print("The goal has already finished or it was already canceled")
      else:
          print(No operation has been done)
```
main() definition:
```
"""
    global pubInfo, subOdom # Global variable definition
    
    time.sleep(1) # Wait for gazebo to be up and running

    rospy.init_node('action_client') # Initialize the node

    clienttar = subsscription to the topic of the server to send the goal

    pubInfo =  publisher for velocity and position

    # Call the 
    subOdom = subscriber from Odom for position and velocity

    # Call the function
    action_client()

if __name__ == '__main__':
    try:
        main() # Call the main function
    except rospy.ROSInterruptException:
        Print error message
```
  







