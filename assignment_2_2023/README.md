 
# Assignment II _ 2023

## ROS Action Server For Navigation 

This project involves the development of a ROS package to enhance the robot navigation system by implementing an action server for long-running tasks. The primary focus is on creating three nodes within a new package to improve user interaction and provide useful information:

Nodes:
(a) Action Client Node
Allows users to set a target (x, y) or cancel it.
Utilizes feedback/status from the action server to determine when the target has been reached.
Publishes robot position and velocity as a custom message (x, y, vel_x, vel_z) based on the /odom topic.

(b) Service Node for Target Coordinates
Provides a service that, when called, returns the coordinates of the last target sent by the user.

(c) Service Node for Distance and Speed Calculation
Subscribes to the robots position and velocity using a custom message.
Implements a server to retrieve the distance of the robot from the target and the robots average speed.
A parameter in the launch file determines the size of the averaging window.


## Dependencies

 actionlib
  actionlib_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf

## Documentation 
 the link to the documentation :
 ```bash
 https://sillyvon.github.io/RT-II-Assignment-Doc/
 ```
## Installation

### create a ROS workspace 
```bash
$ -mkdir\workspace_ros/src
```
### build the ROS workspace 
```bash
$ catkin_make
```
Clone this package in the src folder
```bash
$ git clone https://github.com/SillyVon/RT-Assignment-II-_-2023.git
```
### build the ROS wokrspace again 
```bash
$ catkin_make
```
### run the package by running the launcher in the package 
```bash
$ roslaunch assignment_2_2023 assignment1.launch
```

## Running the nodes 

in order to run the nodes and activate the services, use the rosservice calls in a seperated terminal with the argument True .

### node last target service 
```bash
$ rosservice call last_target True
```
### node velocity and distance 

```bash
$ rosservice call distance_from_goal True
```

## Algorithm Overview

This section provides a high-level overview of the algorithm used in the code. Below is a pseudocode representation of the main functionalities.

### Initialization and Setup

```pseudocode
# Initialize ROS node and action client
rospy.init_node('user_interface')
client = new SimpleActionClient('/reaching_goal', PlanningAction)
client.wait_for_server()

# Set up publishers and subscribers
robot_state_publisher = new Publisher('/Coordinates_velocity', Coordinates_velocity, queue_size=1)
odom_subscriber = new Subscriber('/odom', Odometry, odom_callback, robot_state_publisher, queue_size=1)

### Goal Setting and User Interaction

WHILE not rospy.is_shutdown():
    rospy.sleep(1)
    x = GET_USER_INPUT("Enter x coordinate for the goal: ")
    y = GET_USER_INPUT("Enter y coordinate for the goal: ")
    set_goal(client, x, y)

    WHILE not rospy.is_shutdown():
        rospy.sleep(1)
        goal_status = client.get_state()

        IF goal_status == GoalStatus.ACTIVE:
            PRINT "Goal is active."
            cancel_input = GET_USER_INPUT("Do you want to cancel the goal? (yes/no): ").lower()

            IF cancel_input == "yes":
                success = cancel_goal(client)
                IF success:
                    PRINT "Goal successfully canceled."
                    BREAK
                ELSE:
                    PRINT "Failed to cancel goal."
                    BREAK
        ELIF goal_status == GoalStatus.SUCCEEDED:
            PRINT "Goal is reached."
            BREAK
        ELIF goal_status == GoalStatus.ABORTED:
            PRINT "Goal is canceled."
            BREAK
        ELIF goal_status != GoalStatus.ACTIVE:
            PRINT "Failed to set the goal."
            BREAK

    PRINT "Set a new goal."

###Exception Handling

IF __name__ == '__main__':
    TRY:
        main()
    EXCEPT rospy.ROSInterruptException:
        PRINT "Program interrupted before completion."
        PASS

