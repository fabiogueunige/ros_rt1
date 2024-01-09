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

class PlanningAction(object):
    # create messages that are used to publish feedback/result
    _feedback = assignment_2_2023.msg.PlanningFeedback()
    _result = assignment_2_2023.msg.PlanningResult()

    def __init__(self, name):
        # create the action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer("target_server", assignment_2_2023.msg.PlanningAction, goal_callback = self.goal_callback, auto_start= False)
        self._as.start()

    def goal_callback(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        # append the seeds for the fibonacci sequence
        #self._result.sequence = []
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, moving to (%f, %f)' % (self._action_name, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y))
        # start executing the action
        #for i in range(1, goal.order):
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
        # publish the feedback
        self._feedback.feedback = "Moving to (%f, %f)" % (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
        self._as.publish_feedback(self._feedback)
        # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #r.sleep()
        # append the next number in the fibonacci sequence
        #self._result.sequence.append(self._feedback.feedback)
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)