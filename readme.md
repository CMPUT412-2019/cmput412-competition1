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
 velocity is set to be proportional to this (capped at a maximum speed). The robot moves slower backward than forward.
 
 If the robot is moving forward, it also changes direction according to a proportional control on
 the z-angle (again capped and smoothed with a velocity smoother).
 
 ## Nodes
 
 ### target_position_estimator.py
 
 This node estimates and publishes the target's position from the laser scan. It is called automatically by the `run*.launch` files.
 
 #### Published
 
   - **target_pose** (*geometry_msgs/Pose2D*): The estimated 2D pose of the target in the `odom` frame. The angle is not used.
   - **target_point** (*geometry_msgs/PointStamped*): The estimated position of the target. In general it is probably simpler to subscribe to **target_pose**, but this topic is useful for visualization.

#### Subscribed

  - **pose2d** (*geometry_msgs/Pose2D*): The pose of the robot in the `odom` frame.
  - **scan** (*sensor_msgs/LaserScan*): The laser scan data.


#### Parameters

  - **angle_cutoff_offset** (*float*): Controls the angle at which points in the laserscan are ignored. If 0, then all points in the right hemisphere are ignored. If positive, then some points in the right hemisphere are kept (those with angle from the center of less than `angle_cutoff_offset`). If negative, then similarly some points in the left hemisphere are discarded.

### control.py

This node contains the `smach` state machine used to control the robot. It is called automatically by the `run*.launch` files.

#### Published

  - **cmd_vel** (*geometry_msgs/Twist*): The Twist messages used to control the robot

#### Subscribed

  - **joy** (*sensor_msgs/Joy*): Joystick input
  - **odom** (*nav_msgs/Odometry*): Robot odometry
  - **pose2d** (*geometry_msgs/Pose2D*): The robot's pose in the `odom` frame
  - **target** (*geometry_msgs/Pose2D*): The pose of the target to follow, in the `odom` frame (angle is ignored).

#### Parameters

  - **follow_distance** (*float*, default `1.0`): The distance between the target and the midpoint (the point the robot follows)
  - **retreat_speed** (*float*, default `0.1`): Speed at which robot backs up when midpoint is behind it.
  - **angle_switch_distance** (*float*, default `0.1`): Distance to midpoint at which the robot starts turning toward the target instead of toward the midpoint.
  - **linear_speed_cutoff** (*float*, default `0.8`): Maximum linear speed before controller starts to cap the proportional control.
  - **linear_speed_multiplier** (*float*, default `1.0`): Proportional constant for the linear speed.
  - **angular_speed_cutoff** (*float*, default `2.0`): Maximum angular speed before controller starts to cap the proportional control.
  - **angular_speed_multiplier** (*float*, default `6.0`): Proportional constant for the angular speed.
 