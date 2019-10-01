from __future__ import division

import math
import rospy
import numpy as np
from smach import State
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry


def normalize(v):  # type: (np.ndarray) -> np.ndarray
    norm = np.linalg.norm(v)
    if norm == 0:
        return np.zeros_like(v)
    return v/norm


class FollowState(State):
    def __init__(self):
        State.__init__(self, outcomes=['ok'])
        self.odom_subscriber = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.pose_subscriber = rospy.Subscriber('pose2d', Pose2D, self.pose_callback, queue_size=1)
        self.target_subscriber = rospy.Subscriber('target', Pose2D, self.target_callback, queue_size=1)
        self.twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom = None  # type: Odometry
        self.pose = None  # type: Pose2D
        self.target = None  # type: Pose2D
        self.rate = rospy.Rate(10)
        # Distance from midpoint to target
        self.follow_distance = rospy.get_param('~follow_distance', 1.0)
        # Speed at which robot moves when backing up when midpoint is behind it
        self.retreat_speed = rospy.get_param('~retreat_speed', 0.1)
        # Distance at which angle starts tracking target instead of midpoint
        self.angle_switch_distance = rospy.get_param('~angle_switch_distance', 0.1)
        # Maximum linear speed before the controller caps proportional control, and displacement to speed multiplier
        self.linear_speed_cutoff = rospy.get_param('~linear_speed_cutoff', 0.8)
        self.linear_speed_multiplier = rospy.get_param('~linear_speed_multiplier', 1.0)
        # Same thing, but for angular speed
        self.angular_speed_cutoff = rospy.get_param('~angular_speed_cutoff', 2.0)
        self.angular_speed_multiplier = rospy.get_param('~angular_speed_multiplier', 6.0)

    def odom_callback(self, message):
        self.odom = message

    def pose_callback(self, message):
        self.pose = message

    def target_callback(self, message):
        self.target = message

    def execute(self, _):
        while self.odom is None or self.target is None or self.pose is None:
            rospy.logdebug('Waiting for odom and target and pose...')
            rospy.sleep(1)

        while True:
            self.drive_toward_target()
            self.rate.sleep()

    def drive_toward_target(self):
        # Calculate midpoint and target displacement
        dx_tr = self.target.x - self.pose.x
        dy_tr = self.target.y - self.pose.y
        d_tr = np.sqrt(dx_tr**2 + dy_tr**2)

        m_x = self.target.x - dx_tr/d_tr * self.follow_distance
        m_y = self.target.y - dy_tr/d_tr * self.follow_distance

        dx_mr = m_x - self.pose.x
        dy_mr = m_y - self.pose.y
        d_mr = np.sqrt(dx_mr**2 + dy_mr**2)

        # If midpoint is behind the robot, just move backwards slowly
        if (np.cos(self.pose.theta) * dx_mr + np.sin(self.pose.theta) * dy_mr) < 0:
            self.publish_twist(-self.retreat_speed, 0.0)
            return

        # Now, we move toward midpoint, but if close we instead move angle to target
        if d_mr < self.angle_switch_distance:
            dtheta = math.atan2(dy_tr, dx_tr) - self.pose.theta
        else:
            dtheta = math.atan2(dy_mr, dx_mr) - self.pose.theta
        if abs(dtheta) > np.pi:
            dtheta = dtheta - np.sign(dtheta) * 2 * np.pi

        # Calculate the linear/angular speeds from displacements
        dp = np.array([dx_mr, dy_mr])
        if np.linalg.norm(dp) < self.linear_speed_cutoff:
            v = dp * self.linear_speed_multiplier
        else:
            v = dp / np.linalg.norm(dp) * self.linear_speed_cutoff * self.linear_speed_multiplier
        vx = v[0] * math.cos(self.pose.theta) + v[1] * math.sin(self.pose.theta)  # TODO: maybe this should be proejcted as dp instead of v
        if abs(dtheta) < self.angular_speed_cutoff:
            vtheta = dtheta * self.angular_speed_multiplier
        else:
            vtheta = np.sign(dtheta) * self.angular_speed_cutoff * self.angular_speed_multiplier
        self.publish_twist(vx, vtheta)

    def publish_twist(self, vx, vtheta):  # type: (float, float) -> None
        t = Twist()
        t.linear.x = vx
        t.angular.z = vtheta
        self.twist_publisher.publish(t)
