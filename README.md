# Assignment 2
--------------

## Launch files example
```
<launch>
  <node name="turtlesim_node" pkg="turtlesim" type="turtlesim_node">
  <param name="background_b" type="int" value="0"/>
  <param name="background_g" type="int" value="0"/>
  <param name="background_r" type="int" value="255"/>
  </node>
  <node name="service" pkg="turtlebot_controller" type="service_node"/>
  <node name="controller" pkg="turtlebot_controller" type="exercise1" output="screen"/>
</launch>
```
or
```
<launch>
  <node name="turtlesim_node" pkg="turtlesim" type="turtlesim_node"/>
  <param name="turtlesim_node/background_b" type="int" value="0"/>
  <param name="turtlesim_node/background_g" type="int" value="0"/>
  <param name="turtlesim_node/background_r" type="int" value="255"/>
  <node name="service" pkg="turtlebot_controller" type="service_node"/>
  <node name="controller" pkg="turtlebot_controller" type="exercise1" output="screen"/>
</launch>
```

## Test 
```
roslaunch robot_description sim.launch
```
Rviz is a tool for ROS Visualization. It's a 3-dimensional visualization tool for ROS. It allows the user to view the
simulated robot model, log sensor information from the robot's sensors, and replay the logged sensor information.
By visualizing what the robot is seeing, thinking, and doing, the user can debug a robot application from sensor
inputs to planned (or unplanned) actions.
Gazebo is the 3D simulator for ROS
The robot may be controlled using ROS topics (/cmd_vel) (a nice tool is teleop_twist_keyboard, which may be
launched with rosrun teleop_twist_keyboard teleop_twist_keyboard.py). When moving the robot around,
information coming from sensors may be visualized in Rviz (ex: odom, or cameras).

Let’s check more carefully the launch file.
* We add the robot description in the ROS parameter server
* We launch the simulation in an empty world
* We launch the node RVIZ, together with some additional nodes
* We spawn our robots in the simulation

## Step 2 and 3
### Gazebo
* Dynamic simulation based on various physics engines (ODE, Bullet, Simbody and DART)
* Sensors (with noise) simulation
* Plugin to customize robots, sensors and the environment
* Realistic rendering of the environment and the robots
* Library of robot models
* ROS integration

Gazebo is composed by:
* A server gzerver for simulating the physics, rendering and sensors
* A client gzclient that provides a graphical interface to visualize and interact with the simulation
The client and the server communicate using the gazebo communication library
This may be seen by analyzing the launch file included (empty_world.launch in the gazebo_ros package)
* Two different nodes are started, one for the GzServer, and one for the GzClient
* You may also notice all parameters defined in the launch file

### Rviz
When launching Rviz, three nodes are actually executed:
- joint_state_publisher
- robot_state_publisher
- rviz
• joint_state_publisher: the package reads the robot_description parameter from the parameter server, finds all of
the non-fixed joints and publishes a JointState message with all those joints defined. If GUI is present, the
package displays the joint positions in a window as sliders.
• robot_state_publisher: the package uses the URDF specified by the parameter robot_description and the joint
positions from the topic joint_states to calculate the forward kinematics of the robot and publish the results via
tf.
✓ Rviz is executed by specifying a configuration file, which sets the elements that we want to display in the
simulation.
✓ In the example, we specify the fixed frame (odom) and that we want to visualize the robot structure and the
output of the camera.
✓ Topics or visualization elements may be added by selecting them from the add menu.
✓ By selecting “odom” as fixed frame, we may visualize the movement of the robot also in Rviz. This may be more
evident, by adding the visualization of the tf.
✓ The sim2.launch roslaunch file corresponds to the same simulation, but with a slightly different robot: it has a
laser sensor instead of a camera.
✓ The launch file is thus similar to the previous one, but we are now loading a different urdf file as
robot_description parameter in the ROS parameter server, and we are starting Rviz with a different
configuration file: indeed, we are going to visualize the laser sensor instead of the camera output.
✓ Please notice that, differently from images, the laser output may be seen directly in the corresponding frame
✓ Finally sim_w1.launch uses a different environment for the simulation (environments have been stored in the
folder worlds).
✓ Here in the launch file we explicitly launch the gazebo client and the server (we cannot include anymore the
empty_world.launch).
✓ The world has been defined with a default value, so this may be overridden when launching the simulation (es.
roslaunch robot_description sim_w1.launch world:=world01).


✓ Toactually plan the motion of our robot in an environment we need to process the output of the sensors
• The node reading_laser.py converts the 720 readings contained inside the LaserScan msg into five distinct
readings. Each reading is the minimum distance measured on a sector of 60 degrees (total 5 sectors = 180
degrees).
• Moving the robot in the environment we may check if the laser data are correctly updated
```
rosrun robot_description reading_laser.py
```
✓ Let’s now use the information for controlling the robot in the environment. For example, we can let the robot
move around but avoiding obstacles!
✓ obstacle_avoidance.py implements a very simple behaviour: if an obstacle is detected on the front (or front-right
or front-left) rotate until there are no obstacles perceived. If the obstacle is perceived on the right, than rotate on
the left, and viceversa.
```
rosrun robot_description obstacle_avoidance.py
```
The text keep going...[here](https://github.com/fabiogueunige/RosRT1/blob/master/src/assignment_2_2023/pdf/3D_sim_2nd_assignment.pdf)

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






