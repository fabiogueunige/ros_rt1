#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from tf import transformations
from std_srvs.srv import *
import time


# Add the cycle to 
def action_client():
    # Create the connection to the action server.
    client = actionlib.SimpleActionClient('target_server',assignment_2_2023.msg.PlanningAction)
    # Wait until the action server is up and running.
    client.wait_for_server()

    goal = assignment_2_2023.msg.PlanningGoal() # Check
    # Define the goal
    goal.target_pose.pose.position.x = 10
    goal.target_pose.pose.position.y = 10
    # Send the goal
    client.send_goal(goal)
    # Wait for the result
    client.wait_for_result()
    # Return the result
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('action_client')
        # Call the function
        result = action_client()
        # Print the result given by the action server
        print("Result: ",result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
        pass