#!/usr/bin/env python

import numpy as np
import rospy
import tf
from geometry_msgs.msg import PointStamped, Pose2D
from sensor_msgs.msg import LaserScan


class TargetPositionEstimator:
    def __init__(self):
        rospy.init_node('target_position_estimator')
        self.target_pose_publisher = rospy.Publisher('target_pose', Pose2D)
        self.target_point_publisher = rospy.Publisher('target_point', PointStamped)
        self.tf_listener = tf.TransformListener()
        self.pose_subscriber = rospy.Subscriber('pose2d', Pose2D, self.pose_callback)
        self.scan_subscriber = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.pose = None  # type: Pose2D
        self.angle_cutoff_offset = rospy.get_param('~angle_cutoff_offset')

    def pose_callback(self, message):
        self.pose = message

    def scan_callback(self, scan):  # type: (LaserScan) -> None
        if self.pose is None:
            rospy.loginfo('Waiting for pose...')
            return
        scan = scan  # type: LaserScan
        
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, num=len(scan.ranges), endpoint=True)
        angle_mid = (scan.angle_min + scan.angle_max) / 2.0
        ranges[angles <= (angle_mid - self.angle_cutoff_offset)] = np.nan

        try:
            index = np.nanargmin(ranges)
        except ValueError:
            rospy.loginfo('No points in scan')
            return
        min_range = ranges[index]
        min_angle = angles[index]
        # Convert to a point
        out_point = PointStamped()
        out_point.header.frame_id = scan.header.frame_id
        out_point.point.x = min_range * np.cos(min_angle)
        out_point.point.y = min_range * np.sin(min_angle)
        self.target_point_publisher.publish(out_point)
        out_point = self.tf_listener.transformPoint(target_frame='/odom', ps=out_point)
        target_pose = Pose2D()
        target_pose.x = out_point.point.x
        target_pose.y = out_point.point.y
        self.target_pose_publisher.publish(target_pose)


if __name__ == '__main__':
    e = TargetPositionEstimator()
    rospy.spin()
