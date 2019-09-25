import math
import rospy
import numpy as np
from smach import State
from geometry_msgs.msg import Pose2D, Twist


class FollowState(State):
    def __init__(self):
        State.__init__(self, outcomes=['ok'])
        self.pose_subscriber = rospy.Subscriber('pose2d', Pose2D, self.pose_callback, queue_size=1)
        self.target_subscriber = rospy.Subscriber('target', Pose2D, self.target_callback, queue_size=1)
        self.twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pose = None  # type: Pose2D
        self.target = None  # type: Pose2D
        self.rate = rospy.Rate(10)
        self.speed = rospy.get_param('~move_speed')
        self.angular_speed = rospy.get_param('~turn_speed')
        self.position_threshold = rospy.get_param('~position_threshold')

    def pose_callback(self, message):
        self.pose = message

    def target_callback(self, message):
        self.target = message

    def execute(self, _):
        while self.pose is None or self.target is None:
            rospy.logdebug('Waiting for odom and target...')
            rospy.sleep(1)

        while True:
            self.drive_toward_target()
            self.rate.sleep()

    def drive_toward_target(self):
        dp, dtheta = self.get_deltas_to_target(self.target)
        if np.linalg.norm(dp) < self.position_threshold:
            return
        if np.linalg.norm(dp) < self.speed:
            return
        v = dp / np.linalg.norm(dp) * self.speed
        vtheta = dtheta if dtheta < self.angular_speed else np.sign(dtheta) * self.angular_speed
        self.publish_twist(v, vtheta)
        self.rate.sleep()

    def get_deltas_to_target(self, target):  # type: (Pose2D) -> Tuple[np.ndarray, float]
        pose = self.pose

        dx = target.x - pose.x
        dy = target.y - pose.y
        dtheta = math.atan2(dy, dx) - pose.theta
        if abs(dtheta) > np.pi:
            dtheta = dtheta - np.sign(dtheta) * 2 * np.pi
        return np.array([dx, dy]), dtheta

    def publish_twist(self, v, vtheta):  # type: (np.ndarray, float) -> None
        t = Twist()
        t.linear.x = max(0, v[0] * math.cos(self.pose.theta) + v[1] * math.sin(self.pose.theta))
        t.angular.z = vtheta
        self.twist_publisher.publish(t)
