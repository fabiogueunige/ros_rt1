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
clienttar = None
pubInfoFeet = None
servCancGoal = None


def insertNumber():
    """
    Function to take the input from the user to know the coordinates of the goal

    Returns
    -------
    inpx : float
        x coordinate of the goal    
    inpy : float    
        y coordinate of the goal
    """
    while(1):
        inpx = input("x: ")
        try:
            inpx = float(inpx)
            break
        except:
            print("Please insert a valid number")
    while(1):
        inpy = input("y: ")
        try:
            inpy = float(inpy)
            break
        except:
            print("Please insert a valid number")
    return inpx, inpy

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


def goal_canceling_service(req):
    """
    Service to cancel the goal
    
    Args:
    req (CancelGoal): request to cancel the goal
    """
    
    global servCancGoal

    rospy.wait_for_service('/cancel_goal')

    response = CancelGoalResponse()

    
    try: 
        clienttar.cancel_goal() 
        response.stat = "Done"
    except:
        response.stat = "Error"
    
    return response




def action_client():
    """
    Function to implement the action client
    """

    global clienttar, servCancGoal

    # Wait until the action server is up and running.
    clienttar.wait_for_server()
    

    # User interface
    print("Hi, welcome to the robot planner simulation")
    print("Please choose the position you want to reach with the robot")
    print("Insert the coordinates to reach as ")
    
    # Create a goal to send (to the action server)
    goal = assignment_2_2023.msg.PlanningGoal()

    # Implements the coordinates acquisition
    inpx, inpy = insertNumber()

    # Update the goal
    goal.target_pose.pose.position.x = inpx
    goal.target_pose.pose.position.y = inpy

    # Sending the goal
    clienttar.send_goal(goal)
    print("You sent the goal with: X = ", goal.target_pose.pose.position.x," Y = ", goal.target_pose.pose.position.y)

    while not rospy.is_shutdown(): 
        # Input from the user
        choice = input("Do you want to insert a new goal? (y/n) ")
        if (choice == 'y'):
            # Implements the coordinates acquisition
            inpx, inpy = insertNumber()

            # Update the goal
            goal.target_pose.pose.position.x = inpx
            goal.target_pose.pose.position.y = inpy

            # Send the goal
            clienttar.send_goal(goal)
            print("You sent the goal with: X = ", goal.target_pose.pose.position.x," Y = ", goal.target_pose.pose.position.y)
        else:
            choice = input("Do you want to cancel the previouse goal and restore ? (y/n) ")
            if (choice == 'y'):
                if (clienttar.get_state() == GoalStatus.ACTIVE):
                    print("You canceled the goal with: X = ", goal.target_pose.pose.position.x," Y = ", goal.target_pose.pose.position.y)
                    clienttar.cancel_goal() 
                else:
                    print("The goal has already finished or it was already canceled")
            else:
                print("No Operations has been done")

def main():
    """
        This function initializes the nodes and calls the action client function.
        This also initializes the publisher and subscriber for the robot information, the service for the goal canceling and the action to reach the goal.
    """
    global pubInfo, subOdom, clienttar, pubInfoFeet, servCancGoal
    
    # Wait for gazebo to be up and running
    time.sleep(1)

    # Initialize the node
    rospy.init_node('action_client')

    # Create the service Client
    clienttar = actionlib.SimpleActionClient('/reaching_goal',assignment_2_2023.msg.PlanningAction)
    """ Client for the action server reaching_goal
    """

    # Service to cancel the goal
    servCancGoal = rospy.ServiceProxy('/cancel_goal', CancelGoal) #  goal_canceling_service va aggiunto?
    """ Service to cancel the goal
    """

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

    # Call the function
    action_client()

if __name__ == '__main__':
    try:
        # Call the main function
        main()
    except rospy.ROSInterruptException:
        # Print error message
        print("Errors in action_client.py")
        print("program interrupted before completion for errors", file=sys.stderr)
        pass
