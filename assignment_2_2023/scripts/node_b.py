#! /usr/bin/env python
""""
.. module::Node_B last_target_service_node
    
    :platform: Unix

    :synppsis: This module is a ROS node that subscribes to the '/reaching_goal/goal' topic and stores the last target coordinates set by the user. It also provides a service '/last_target' that returns the last target coordinates when called.

.. moduleauthor:: Yazan Kayali

subscribes to: 
    /reaching_goal/goal

publishes to: 
    None

services:
    /last_target

clients:
    None

"""

import rospy
from assignment_2_2023.msg import PlanningActionGoal
from geometry_msgs.msg import Point, Pose
from std_srvs.srv import *

# Global defualt variables 
last_target = Point()
last_target.x = rospy.get_param('des_pos_x')
last_target.y = rospy.get_param('des_pos_y')
last_target.z = 0
# Service active state
active_ = False


def callback(msg):
    """
    Callback function 

    This function is called whenever a message is published to the '/reaching_goal/goal' topic. It stores the last target coordinates set by the user.

    Args:
        msg (PlanningActionGoal): The message published to the '/reaching_goal/goal' topic. It contains the target coordinates set by the user.
    """

    global last_target
    #store the last target
    last_target.x = msg.goal.target_pose.pose.position.x
    last_target.y = msg.goal.target_pose.pose.position.y 

def last_target_handler(req):
    """
    Service handler function

    This function is called whenever the service '/last_target' is called. It returns the last target coordinates set by the user.

    Args:
        req (SetBoolRequest): The request message sent to the service '/last_target'. It contains the active state of the service.

    """

    global last_target
    
    # print the last target coordinates

    print(f"\nLast Target set by the user: x = {last_target.x:.4f}, y = {last_target.y:.4f}")

    # Return the status of the service call
    active_ = req.data
    response = SetBoolResponse()
    response.success = True
    response.message = 'Done!'
    return response

def main():
    """
    Main function

    This function initializes the node 'last_target_service_node', subscribes to the '/reaching_goal/goal' topic, and creates the service '/last_target'.

    """

    # Initialize the node

    rospy.init_node('last_target_service_node')

    #subscribe to the '/reaching_goal/goal' topic
    
    sub = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, callback)
    
    #create the service 

    rospy.Service('/last_target', SetBool, last_target_handler)
    

    rospy.spin()

if __name__ == '__main__':
    main()

