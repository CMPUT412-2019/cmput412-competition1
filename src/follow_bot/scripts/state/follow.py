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

        dx_tr = self.target.x - self.pose.x
        dy_tr = self.target.y - self.pose.y

        d_tr = np.sqrt(dx_tr**2 + dy_tr**2)
        min_distance_to_target = 1.0

        m_x = self.target.x - dx_tr/d_tr * min_distance_to_target
        m_y = self.target.y - dy_tr/d_tr * min_distance_to_target

        # Now, we move toward m, but if close we instead move angle to t.

        dx_mr = m_x - self.pose.x
        dy_mr = m_y - self.pose.y
        d_mr = np.sqrt(dx_mr**2 + dy_mr**2)
        if d_mr < 0.1:
            dtheta = math.atan2(dy_tr, dx_tr) - self.pose.theta
        else:
            dtheta = math.atan2(dy_mr, dx_mr) - self.pose.theta
        if abs(dtheta) > np.pi:
            dtheta = dtheta - np.sign(dtheta) * 2 * np.pi

        dp = np.array([dx_mr, dy_mr])

        self.speed = 0.5
        self.angular_speed = 2.0
        v = dp if np.linalg.norm(dp) < self.speed else dp / np.linalg.norm(dp) * self.speed
        vtheta = dtheta if abs(dtheta) < self.angular_speed else np.sign(dtheta) * self.angular_speed

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
