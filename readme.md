# Follow-bot

*CMPUT 412 competition 1*

## Overview
This project is a ROS package that allows a robot to follow another robot that is moving in
a counter-clockwise circular path.

## Setup and prerequisites
Requires a Turtlebot 2 with the RGB-D camera in its default position, and Python 2.7 with ROS
Kinetic. Gazebo is required if one wants to test the project in a simulated environment (although
it must be noted that the robot parameters are currently tuned for a real-world turtlebot and
perform sub-optimally on the simulator).

To get started, simply clone the repository:

    git clone https://github.com/alex-epp/cmput412-competition1
    
The project can be built as follows:
    
    cd cmput412-competition1
    catkin_make
    source devel/setup.bash

To run the project in Gazebo:

    roslaunch follow_bot run_sim.launch

If you want to run the project on a physical Turtlebot, you must plug in the RGB-D camera and the
Kobuki base into your computer, then run

    roslaunch follow_bot run.launch

## Method
The task of following another robot is split into several subtasks.

### Finding the target robot
The first step the follow-bot performs is to locate the robot it is tracking. This is done by means
a virtual laser scan (obtained from the camera's RGB-D information from the `depthimage_to_laserscan`
package). The closest point in the laser scan that has angle <= 0.25 (where pi/2 to the right and
 -pi/2 is to the left) is considered the location of the robot. This point is transformed into the
 odometry frame for the next step.
 
 ### Calcuating a target point
 
 The next step is to locate a target to move towards. This is not actually the location of the target
 robot--instead it is a point in between the follow-bot and the target robot. This is to prevent the
 follow-bot from colliding with the target, and also because when the target is farther away from the
 follow-bot it is in a wider portion of the follow-bot's field of view, reducing the chances of
 losing the target if it moves laterally to the follow-bot's view direction. The target point is 1
 meter behind the target robot.
 
 ### Moving toward the target point
 
 The follow-bot is directed toward the target point by simple proportional control. The difference
 in the forward direction from the target point to the robot is calculated, and the robot's forward
 velocity is set to be proportional to this (capped at a maximum speed, and smoothed with a
 velocity smoother to a maximum acceleration). The robot moves slower backward than forward.
 
 If the robot is moving forward, it also changes direction according to a proportional control on
 the z-angle (again capped and smoothed with a velocity smoother).