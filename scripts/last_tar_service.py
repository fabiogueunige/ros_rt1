#! /usr/bin/env python3

import rospy
from assignment_2_2023 import srv
from assignment_2_2023 import msg
from assignment_2_2023.srv import LastTarget, LastTargetResponse
from assignment_2_2023.msg import PlanningActionGoal, PlanningGoal


subgoal = None
servicetarget = None

# /reaching_goal/goal
def target_service_callback(request):
    """
    Callback function for the service server of the last target

    Parameters
    ----------
    request : LastTarget
        request from the service server

    Returns
    -------
    LastTargetResponse
        response from the service server
    """
    global subgoal
    global postarx, postary

    # Create the response
    #resp = LastTargetResponse()
    #resp.tar_pose_x = postarx
    #resp.tar_pose_y = postary

    # Return the response
    return LastTargetResponse(postarx, postary)
    
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

    postarx = msg.goal.target_pose.pose.position.x
    postary = msg.goal.target_pose.pose.position.y


def last_tar_service():
    """
    Main function for the service server of the last target
    """
    global subgoal, servicetarget

    # Initialize the node
    rospy.init_node('last_target')

    # Create the subscriber
    subgoal = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, target_sub_callback)
    # Create the service
    servicetarget = rospy.Service('/last_target', LastTarget, target_service_callback)
    
    # Spin
    rospy.spin()


if __name__ == '__main__':
    try:
        # Initialize the node
        last_tar_service()
        # Print the result given by the action server
    except rospy.ROSInterruptException:
        print("Errors in last_tar_service.py")
        print("program interrupted before completion for errors", file=sys.stderr)
        pass