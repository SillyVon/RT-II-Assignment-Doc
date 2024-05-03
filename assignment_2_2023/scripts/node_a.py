#! /usr/bin/env python
""""
.. module::Node_A User interface 
    
    :platform: Unix

    :synppsis: This module is the user interface for the robot controller. It allows the user to set a goal for the robot to reach and to cancel the goal if needed. The user can also request the robot's current position and velocity.

.. moduleauthor:: Yazan Kayali

subscribes to: 
    /odom

Publishes to:
    /Robot_state

client : 
    /reaching_goal

"""

from __future__ import print_function
import rospy
# Brings in the SimpleActionClient
import actionlib
import actionlib.msg
# Brings in the messages used by the Planning action including the goal message and the result message.
import assignment_2_2023.msg
from assignment_2_2023.msg import RobotState, PlanningFeedback
from geometry_msgs.msg import Twist, Point, Pose
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry



def send_goal(client, x, y):
    """
    Sending goal function

    This function takes the arguments and sends the goal to the action server

    Args: 
        client (SimpleActionClient): the client object used to send the goal to the action server
        x (float): the x coordinate of the goal
        y (float): the y coordinate of the goal
    """

    # Creates a goal to send to the action server.
    goal = assignment_2_2023.msg.PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # Sends the goal to the action server.
    client.send_goal(goal)


def goal_cancel_(client):
    """
    Cancelling goal function

    This function cancels the goal that has been sent to the action server

    Args: 
        client (SimpleActionClient): the client object used to send the goal to the action server
    """

    # send a cancel goal request
    client.cancel_goal()

    # Wait for the cancellation to be processed
    cancel_state = client.wait_for_result()

    if cancel_state:
        print("Goal has been cancelled.")
    else:
        print("Failed to cancel the goal")


def odom_callback(odom_msg, robot_state_pub):

    """
    Odometry callback function

    This function extracts the position and velocity information from the odometry message and publishes it on the RobotState topic

    Args:
        odom_msg (Odometry): the odometry message containing the position and velocity information
        robot_state_pub (Publisher): the publisher object used to publish the position and velocity information on the RobotState topic
    """

     # Extract position and velocity information from the odom message
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    vel_x = odom_msg.twist.twist.linear.x
    vel_z = odom_msg.twist.twist.angular.z

    # Publish the position and velocity on the RobotState topic
    # Write the message
    robot_state_msg = RobotState(x=x, y=y, vel_x=vel_x, vel_z=vel_z) 
    # Publish the message   
    robot_state_pub.publish(robot_state_msg)  

   

def main ():
    """
    Main function

    This function initializes the node, creates the SimpleActionClient, and subscribes to the odometry topic. It also contains the main loop that allows the user to set a goal for the robot to reach and to cancel the goal if needed. The user can also request the robot's current position and velocity.

    """

    rospy.init_node('robot_controller')

    # Create the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('/reaching_goal',assignment_2_2023.msg.PlanningAction)
    """ instance of the SimpleActionClient class that will be used to send goals to the action server.
    """
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # publisher to the Robot_State custom message 
    robot_state_pub = rospy.Publisher('/Robot_state', RobotState, queue_size=10)
    """ instance of the Publisher class that will be used to publish the robot's position and velocity information.
    """

    # subscriber to the odometry topic
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback, robot_state_pub, queue_size=1)
    """ instance of the Subscriber class that will be used to subscribe to the odometry topic and get the robot's position and velocity information.
    """

    while not rospy.is_shutdown():

        rospy.sleep(1)

        #set a goal
        print ("Set a goal !")

        x = float(input("Enter the x coordinate: "))
        y = float(input("Enter the y coordinate: "))

        send_goal(client, x, y)
        
        while not rospy.is_shutdown():

            rospy.sleep(1)
            #check the status of the goal
            goal_status = client.get_state()

            # if goal status is active 
            if goal_status == GoalStatus.ACTIVE:
                print("Goal status update : active.")
                # ask the user if they would like to cancel the goal
                cancel_goal = input("press'c'to cancel the goal\npress's'for robot status update : ")

                if cancel_goal == 'c':
                   print("Cancelling the goal")
                   goal_cancel_(client)
                   break
                  
                elif cancel_goal == 's':
                    continue
                
                else :
                    print("Invalid input")
                    continue 

            # if goal status is succeeded
            elif goal_status == GoalStatus.SUCCEEDED:
                print("Goal status update : succeeded.")
                break

    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
        pass


