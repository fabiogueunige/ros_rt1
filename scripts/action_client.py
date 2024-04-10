#! /usr/bin/env python3

"""
.. module:: action_client
    :platform: Unix
    :synopsis: Python module for the action client

.. moduleauthor:: Fabio Guelfi fabio.guelfi@libero.it

This module is the action client for the assignment 2 of the course 2023. 
It is used to send the goal to the action server and to cancel the goal if needed. 
It also publishes the position and velocity of the robot.

Subscribes to:
    /odom

Publishes to:
    /robot_info
    /robot_info_feet

Service:
    /cancel_goal

Client:
    /reaching_goal
"""

import rospy
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import Robotinfo, Robotinfofeet
from actionlib_msgs.msg import GoalStatus
from assignment_2_2023.srv import CancelGoal, CancelGoalResponse
import time

# Global variables
pubInfo = None
subOdom = None  
# clienttar = None
pubInfoFeet = None
# servCancGoal = None

def odom_callback(msg):
    """
    Callback function for the subscriber of the odometry

    Args:
    msg (Odometry): message from the subscriber for the odometry
    """
    global pubInfo, pubInfoFeet

    # Create the message to publish
    msgInfo = Robotinfo()
    msgInfo.pos_x = round(msg.pose.pose.position.x,3)
    msgInfo.pos_y = round(msg.pose.pose.position.y,3)
    msgInfo.vel_x = round(msg.twist.twist.linear.x,3)
    msgInfo.vel_ang_z = round(msg.twist.twist.angular.z,3)

    # Publish the message
    pubInfo.publish(msgInfo)

    msgFeet = Robotinfofeet()
    msgFeet.feet_x = round(msg.pose.pose.position.x*3.28,3)
    msgFeet.feet_y = round(msg.pose.pose.position.y*3.28,3)

    pubInfoFeet.publish(msgFeet)


def main():
    """
        This function initializes the nodes and calls the action client function.
        This also initializes the publisher and subscriber for the robot information, the service for the goal canceling and the action to reach the goal.
    """
    global pubInfo, subOdom, pubInfoFeet
    
    # Wait for gazebo to be up and running
    time.sleep(1)

    # Initialize the node
    rospy.init_node('action_client') 

    # Call the publisher for velocity and position
    pubInfo =  rospy.Publisher('/robot_info', Robotinfo, queue_size=1)
    """ Publisher for the robot information
    """

    # Call the subscriber from Odom for position and velocity
    subOdom = rospy.Subscriber('/odom', Odometry, odom_callback)
    """ Subscriber for the odometry
    """

    # Call the publisher for information in feet
    pubInfoFeet = rospy.Publisher('/robot_info_feet', Robotinfofeet, queue_size=1)
    """ Publisher for the robot information in feet
    """

    rospy.spin() # Keeps the node running

if __name__ == '__main__':
    try:
        # Call the main function
        main()
    except rospy.ROSInterruptException:
        # Print error message
        print("Errors in action_client.py")
        print("program interrupted before completion for errors", file=sys.stderr)
        pass
