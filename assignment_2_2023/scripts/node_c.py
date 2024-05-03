#! /usr/bin/env python
""""
.. module::Node_C Distance from Goal
  
    :platform: Unix

    :synppsis: This module calculates the distance from the goal and the average speed of the robot

.. moduleauthor:: Yazan Kayali

subscribes to
    /RobotState: The robot's state information

    /reaching_goal/goal: The goal position


published topics:
    None
    
services:
    distance_from_goal: The service that calculates the distance from the goal and the average speed of the robot

"""

import rospy
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import PlanningActionGoal, RobotState
from geometry_msgs.msg import Point, Pose, Twist
from std_srvs.srv import *
import math

# Global variables
# Initialize the goal position from parameters set in the launch file
goal = Point()
goal.x = rospy.get_param('des_pos_x')
goal.y = rospy.get_param('des_pos_y')
goal.z = 0
# Initialize the current position of the robot
position = RobotState()
# Service activation state
active = False
# List to store previous velocity data
velocities = []

# Get the averaging window size from the launch file
avg_window_size = rospy.get_param('avg_window_size')

# Callbacks
# Callback function for updating robot state information
def state_callback(msg):
    """
    State callback function for updating robot state information

    This function updates the robot's position and velocity information

    Parameters:
        msg (RobotState): The message containing the robot's state information
    """
    global position, velocities, avg_window_size
    # Extract position and velocity information from the RobotState message
    position.x = msg.x
    position.y = msg.y
    position.vel_x = msg.vel_x
    position.vel_z = msg.vel_z
    
    # Add current velocity to the list
    velocities.append(position.vel_x)
    # Maintain only the latest velocity samples within the window
    if len(velocities) > avg_window_size:
    	velocities.pop(0) # Remove the oldest velocity in the list

# Callback function for updating the goal position
def goal_callback(msg):
    """
    Goal callback function for updating the goal position

    This function updates the goal position

    Parameters:
        msg (PlanningActionGoal): The message containing the goal position
    """
    global goal
    goal.x = msg.goal.target_pose.pose.position.x
    goal.y = msg.goal.target_pose.pose.position.y     
    
# Service callback function    
def distance_from_goal_handler(req):
    """
    Service callback function for calculating the distance from the goal

    This function calculates the distance from the goal and the average speed of the robot

    Parameters:
        req (SetBool): The service request message
    """

    global goal, position, velocities, active
    
    # Declare distance as a local variable
    distance = Point()
    
    # Calculate distance from the goal
    distance.x = goal.x - position.x
    distance.y = goal.y - position.y
    # Display the distance from the goal
    print(f"\nDistance from the goal: x = {distance.x:.4f}, y = {distance.y:.4f}")
    
    # Calculate the average speed
    # Check if there are values in the velocities list
    if velocities:
        avg_speed = sum(map(abs, velocities)) / len(velocities)
    else: 
        avg_speed = 0.0		
    # Print the average speed
    print(f"Average speed: v = {avg_speed:.4f}")

    # Return the status of the service call
    active = req.data
    response = SetBoolResponse()
    response.success = True
    response.message = 'Done!'
    return response    

def main():
    """
    Main function for the distance_from_goal node

    This function initializes the node, subscribes to the RobotState and PlanningActionGoal topics, and initializes the service
    """
    # Initialize the node	
    rospy.init_node('distance_from_goal')
    
    # Subscribe to the RobotState topic
    sub_odom = rospy.Subscriber('/RobotState', RobotState, state_callback)
    """" instantiate the subscriber to the /RobotState topic, and the callback function is state_callback
    """
    # Subscribe to the reaching_goal/goal topic
    sub_goal = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, goal_callback)
    """ instantiate the subscriber to the /reaching_goal/goal topic, and the callback function is goal_callback
    """
    # Initialize the service
    srv = rospy.Service('distance_from_goal', SetBool, distance_from_goal_handler)
    
    rospy.spin()

if __name__ == '__main__':
    main()