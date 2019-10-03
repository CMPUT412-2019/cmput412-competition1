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

### Finding the robot
The first step the follow-bot performs is to locate the robot it is tracking. This is done by means
a virtual laser scan (obtained from the camera's RGB-D information from the `depthimage_to_laserscan`
package). 
