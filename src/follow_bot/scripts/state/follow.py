from __future__ import division

import math
import rospy
import numpy as np
from smach import State
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry

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
        self.speed = rospy.get_param('~move_speed')
        self.angular_speed = rospy.get_param('~turn_speed')
        self.position_threshold = rospy.get_param('~position_threshold')

    def odom_callback(self, message):
        self.odom = message

    def pose_callback(self, message):
        self.pose = message

    def target_callback(self, message):
        self.target = message
        # self.target = Pose2D()
        # self.target.x = 10
        # self.target.y = 10

    def execute(self, _):
        while self.odom is None or self.target is None or self.pose is None:
            rospy.logdebug('Waiting for odom and target and pose...')
            rospy.sleep(1)

        while True:
            self.drive_toward_target()
            self.rate.sleep()

    def drive_toward_target(self):
        dp, dtheta, ddtheta = self.get_deltas_to_target(self.target)
        # if np.linalg.norm(dp) < self.position_threshold:
        #     return
        # if np.linalg.norm(dp) < self.speed:
        #     return
        self.speed = 1
        v = dp if np.linalg.norm(dp) < self.speed else dp / np.linalg.norm(dp) * self.speed
        # v = dp * 0.01 - np.array([self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y]) * 0.0

        if np.linalg.norm(dp) < 0.01:
            v *= 0.0

        Kp = 2.0
        Kd = 0.0
        vtheta = dtheta if abs(dtheta) < 2.0 else np.sign(dtheta) * 2.0 #  Kp * dtheta + Kd * ddtheta

        self.publish_twist(v, vtheta)

    def get_deltas_to_target(self, target):  # type: (Pose2D) -> Tuple[np.ndarray, float, float]
        odom = self.odom

        dx = target.x - odom.pose.pose.position.x
        dy = target.y - odom.pose.pose.position.y
        dtheta = math.atan2(dy, dx) - self.pose.theta
        if abs(dtheta) > np.pi:
            dtheta = dtheta - np.sign(dtheta) * 2 * np.pi
        ddtheta = -odom.twist.twist.angular.z
        return np.array([dx, dy]), dtheta, ddtheta

    def publish_twist(self, v, vtheta):  # type: (np.ndarray, float) -> None
        t = Twist()
        theta = self.pose.theta
        t.linear.x = max(0, v[0] * math.cos(theta) + v[1] * math.sin(theta))
        t.angular.z = vtheta
        self.twist_publisher.publish(t)
