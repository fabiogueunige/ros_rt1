#! /usr/bin/env python3

import rospy
#from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import Robotinfo
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningResult, PlanningActionResult
from actionlib_msgs.msg import GoalStatus
import time

# Global variables
pubInfo = None
subOdom = None  

# AGGIUNGI ROUND AI MESSAGGI (TUTTI DIREI) POS E VEL PER EVITARE DI PUBBLICARE NUMERI TROPPPO LUNGHI
# AGGIUNGI LA STAMPA DEL FEEDBACK AL GOAL
# SCEGLI SE DARE LA IMPOSTAZIONE DI DARE UN NUOVO TARGET ANCHE QUANDO UNO E GIA PRESENTE
# CONTROLLA CHE FUNZIONI DAVVERO LA CARTELLA GIT ANCHE SE CAMBI IL NOME DELLA CARTELLA PER GIT


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
            # to exit form the while loop
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

    Parameters
    ----------
    msg : Odometry
        message from the subscriber
    """
    global pubInfo

    # Create the message to publish
    msgInfo = Robotinfo()
    msgInfo.pos_x = round(msg.pose.pose.position.x,3)
    msgInfo.pos_y = round(msg.pose.pose.position.y,3)
    msgInfo.vel_x = round(msg.twist.twist.linear.x,3)
    msgInfo.vel_ang_z = round(msg.twist.twist.angular.z,3)

    # Publish the message
    pubInfo.publish(msgInfo)

def action_client():
    """
    Function to implement the action client
    """

    # Create the connection to the action server.
    clienttar = actionlib.SimpleActionClient('/reaching_goal',assignment_2_2023.msg.PlanningAction)
    
    # Wait until the action server is up and running.
    clienttar.wait_for_server()

    # User interface
    print("Hi, welcome to the robot planner simulation")
    print("Please choose the position you want to reach with the robot")
    print("Insert the coordinates to reach as ")
    
    # Create a goal to send (to the action server)
    goal = assignment_2_2023.msg.PlanningGoal()

    # implements the coordinates acquisition
    inpx, inpy = insertNumber()

    # Update the goal
    goal.target_pose.pose.position.x = inpx
    goal.target_pose.pose.position.y = inpy

    # Sending the goal
    clienttar.send_goal(goal)
    print("You sent the goal with: X = ", goal.target_pose.pose.position.x," Y = ", goal.target_pose.pose.position.y)

    while not rospy.is_shutdown(): # can sobstitute with rospy.spin()
        # Input from the user
        choice = input("Do you want to insert a new goal? (y/n)")
        if (choice == 'y'):
            # implements the coordinates acquisition
            inpx, inpy = insertNumber()

            # Update the goal
            goal.target_pose.pose.position.x = inpx
            goal.target_pose.pose.position.y = inpy

            # Send the goal
            clienttar.send_goal(goal)
            print("You sent the goal with: X = ", goal.target_pose.pose.position.x," Y = ", goal.target_pose.pose.position.y)
        else:
            choice = input("Do you want to cancel the previouse goal and restore ? (y/n)")
            if (choice == 'y'):
                # add the control to the status of the goal!!
                if (clienttar.get_state() == GoalStatus.ACTIVE):
                    print("You canceled the goal with: X = %f, Y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
                    clienttar.cancel_goal() 
                else:
                    print("The goal has already finished or it was already canceled")
            else:
                print("The goal is still active with: X = ", goal.target_pose.pose.position.x," Y = ", goal.target_pose.pose.position.y)
        
   
    # Return the result
    # return client.get_result()

def main():
    """
    Main function
    """
    global pubInfo, subOdom
    
    # Wait for gazebo to be up and running
    time.sleep(1)

    # Initialize the node
    rospy.init_node('action_client')

    # Call the publisher for velocity and position
    pubInfo =  rospy.Publisher('/robot_info', Robotinfo, queue_size=1)

    # Call the subscriber from Odom for position and velocity
    subOdom = rospy.Subscriber('/odom', Odometry, odom_callback)

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