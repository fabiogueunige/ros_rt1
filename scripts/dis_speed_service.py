#! /usr/bin/env python3

"""
.. module:: dis_speed_service
    :platform: Unix
    :synopsis: Python module for the service server of the distance and speed

.. moduleauthor:: Fabio Guelfi

This module is the service server for the assignment 2 of the course 2023.
It is used to calculate the distance from the target and the average speed of the robot.
It also subscribes to the robot info and to the goal to calculate the distance and the speed.

Subscribes to:
    /robot_info
    /reaching_goal/goal

Service:
    /dis_speed

Parameters:
    /wd_size
"""
import rospy
from nav_msgs.msg import Odometry
import actionlib.msg
import math
import assignment_2_2023.srv
from assignment_2_2023.msg import Robotinfo
from assignment_2_2023.msg import PlanningActionGoal
from actionlib_msgs.msg import GoalStatus
from assignment_2_2023.srv import SpeedDistance, SpeedDistanceResponse
import time

subInfo = None
servicedis_speed = None
subgoal = None
listspeedx = []
listspeedz = []

def list_update(val, listv):
    """
    Function to update the list of the velocities

    Parameters
    ----------
    val : float
        value to append to the list
    """
    wdsize = rospy.get_param('/wd_size')
    """Get the value of the parameter wd_size
    """

    # Append the value to the list
    listv.append(val)
    # If the list is too long, remove the first element
    if len(listv) > wdsize:
        listv.pop(0)

    

def dis_speed_service_callback(request):
    """
    Callback function for the service server of the distance and speed

    Args:
    request (SpeedDistance): request from the service server of the distance and speed

    Returns:
    SpeedDistanceResponse: response from the service server of the distance and speed
    """
    global servicedis_speed
    global px, py
    global postarx, postary
    global listspeedx
    global listspeedz

    # Avarage distance from the target
    try:
        avg = math.sqrt((px-postarx)**2 + (py-postary)**2)
    except:
        rospy.loginfo("Error in the calculation of the average distance")
        rospy.loginfo("The target has not been initialized yet")
        avg = 0
    resp = SpeedDistanceResponse()
    # Add the distance to the response
    resp.distance_target = round(avg,3)

    # Avarage speed on x
    try:
        # Claculate the avarege distance
        avg = sum(listspeedx)/len(listspeedx)
    except:
        rospy.loginfo("Error in the calculation of the average speedx")
        rospy.loginfo("The target has not been moved yet")
        resp.avgspeedx = 0
    # Add the speed  x to the response
    resp.avg_speed_x = round(avg,3)

    # Avarage speed on z
    try:
        # Claculate the avarege distance
        avg = sum(listspeedz)/len(listspeedz)
    except:
        rospy.loginfo("Error in the calculation of the average speedz")
        rospy.loginfo("The target has not been moved yet")
        avg = 0
    # Add the speed z to the response
    resp.avgspeed_ang_z = round(avg,3)
    
    # Return service
    return resp


def info_callback(msg):
    """
    Callback function for the subscriber of the robot info
    
    Args:
    msg (Robotinfo): message from the subscriber for the robot info
    """
    global px, py
    global listspeedx, listspeedz
    
    # Update the values of the global variables
    px = msg.pos_x
    py = msg.pos_y
    list_update(msg.vel_x, listspeedx)
    list_update(msg.vel_ang_z, listspeedz)
    

def target_sub_callback(msg):
    """
    Callback function for the subscriber of the goal

    Args:
    msg (PlanningActionGoal): message from the subscriber
    """
    global subgoal
    global postarx, postary

    # Savings the values in the global variables
    postarx = msg.goal.target_pose.pose.position.x
    postary = msg.goal.target_pose.pose.position.y

def dis_speed_service():
    """
    Main function for the service server of the distance and speed
    It defines the service server for the speed and the distance and the subscribers to the robot info and to the goal
    """
    global subInfo, servicedis_speed, subgoal
    time.sleep(2)
    rospy.init_node('dis_speed_service')

    # Create the service server
    servicedis_speed = rospy.Service('/dis_speed', SpeedDistance, dis_speed_service_callback)
    """Create the service server for the distance and speed
    """

    # Create the info subscriber
    subInfo = rospy.Subscriber('/robot_info', Robotinfo, info_callback)
    """Create the info subscriber to the robot info
    """

    # Create the goal subscriber or rospy.getparam('des_pos_y')
    subgoal = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, target_sub_callback)
    """Create the goal subscriber to reaching goal
    """
    # Spin
    rospy.spin()


if __name__ == '__main__':
    try:
        # Call the main function
        dis_speed_service()
    except rospy.ROSInterruptException:
        # Print errors
        print("Errors in dis_speed_service.py")
        print("program interrupted before completion for errors", file=sys.stderr)
        pass