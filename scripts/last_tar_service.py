#! /usr/bin/env python3

"""
.. module:: last_tar_service
    :platform: Unix
    :synopsis: Python module for the service server of the last target

.. moduleauthor:: Fabio Guelfi

This module is the service server for the assignment 2 of the course 2023.
It is used to return the last target of the robot.
It also subscribes to the goal to know the last target.

Subscribes to:
    /reaching_goal/goal

Service:
    /last_target
"""

import rospy
from assignment_2_2023 import srv
from assignment_2_2023 import msg
from assignment_2_2023.srv import LastTarget, LastTargetResponse
from assignment_2_2023.msg import PlanningActionGoal
import time

subgoal = None
servicetarget = None

def target_service_callback(request):
    """
    Callback function for the service server of the last target

    Args:
    request(LastTargetRequest): request from the client

    Returns:
    LastTargetResponse: response to the client
    """
    global subgoal
    global postarx, postary

    # Cutting the values to 3 decimals
    postarx = round(postarx,3)
    postary = round(postary,3)
    # Return the response for the service
    return LastTargetResponse(postarx, postary)
    
def target_sub_callback(msg):
    """
    Callback function for the subscriber of the goal

    Args:
    msg(PlanningActionGoal): message from the topic
    """
    global subgoal
    global postarx, postary

    # Savings the values in the global variables
    postarx = msg.goal.target_pose.pose.position.x
    postary = msg.goal.target_pose.pose.position.y


def last_tar_service():
    """
    Main function for the service server of the last target
    It defines the service server for the last target and the subscriber to the goal
    """
    global subgoal, servicetarget

    time.sleep(2)
    # Initialize the node
    rospy.init_node('last_target')

    # Create the subscriber
    subgoal = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, target_sub_callback)
    """Create the goal subscriber to reaching goal
    """
    
    # Create the service
    servicetarget = rospy.Service('/last_target', LastTarget, target_service_callback)
    """Create the service server for the last target
    """

    # Spin
    rospy.spin()


if __name__ == '__main__':
    try:
        # Call the main function
        last_tar_service()
    except rospy.ROSInterruptException:
        # Print the error
        print("Errors in last_tar_service.py")
        print("program interrupted before completion for errors", file=sys.stderr)
        pass