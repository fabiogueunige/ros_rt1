#! /usr/bin/env python3

"""
Another service node that subscribes to the robot’s position and velocity 
(using the custom message) and implements a server to retrieve the distance 
of the robot from the target and the robot’s average speed.
"""
import rospy
#from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import math
import assignment_2_2023.msg
import assignment_2_2023.srv
from assignment_2_2023.msg import Robotinfo
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningResult, PlanningActionGoal
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

    # Append the value to the list
    listv.append(val)
    # If the list is too long, remove the first element
    if len(listv) > wdsize:
        listv.pop(0)

    

def dis_speed_service_callback(request):
    """
    Callback function for the service server of the distance and speed

    Parameters
    ----------
    request : SpeedDistance
        request from the service server
    Returns
    -------
    resp : SpeedDistanceResponse
        response of the service
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
    resp.distance_target = avg

    # Avarage speed on x
    try:
        # Claculate the avarege distance
        avg = sum(listspeedx)/len(listspeedx)
    except:
        rospy.loginfo("Error in the calculation of the average speedx")
        rospy.loginfo("The target has not been moved yet")
        resp.avgspeedx = 0
    # Add the speed  x to the response
    resp.avg_speed_x = avg

    # Avarage speed on z
    try:
        # Claculate the avarege distance
        avg = sum(listspeedz)/len(listspeedz)
    except:
        rospy.loginfo("Error in the calculation of the average speedz")
        rospy.loginfo("The target has not been moved yet")
        avg = 0
    # Add the speed z to the response
    resp.avgspeed_ang_z = avg

    # Avarage speed
    #for i in range(len(listspeedx)):
        #try:
            #avg += math.sqrt((listspeedx[i] + listspeedz[i])**2)
        #except:
            #rospy.loginfo("Error in the calculation of the average speed total")
            #rospy.loginfo("The target has not been moved yet")
            #avg = 0
    #try:
        #avg = avg/len(listspeedx)
    #except:
        #avg = 0
    # Add the speed to the response
    #resp.avgspeedtot = avg
    
    # Return service
    return resp


def info_callback(msg):
    """
    Callback function for the subscriber of the robot info
    
    Parameters
    ----------
    msg : Robotinfo
        message from the subscriber
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

    Parameters
    ----------
    msg : PlanningGoal
        message from the subscriber
    """
    global subgoal
    global postarx, postary

    # Savings the values in the global variables
    postarx = msg.goal.target_pose.pose.position.x
    postary = msg.goal.target_pose.pose.position.y

def dis_speed_service():
    """
    Function to implement the service server
    
    Returns
    -------
    resp : SpeedDistanceResponse
        response of the service"""
    global subInfo, servicedis_speed, subgoal
    time.sleep(2)
    rospy.init_node('dis_speed_service')

    # Create the service server
    servicedis_speed = rospy.Service('/dis_speed', SpeedDistance, dis_speed_service_callback)

    # Create the info subscriber
    subInfo = rospy.Subscriber('/robot_info', Robotinfo, info_callback)

    # Create the goal subscriber or rospy.getparam('des_pos_y')
    subgoal = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, target_sub_callback)

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