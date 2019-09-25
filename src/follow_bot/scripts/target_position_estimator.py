#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D, PointStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import tf


class TargetPositionEstimator:
    def __init__(self):
        rospy.init_node('target_position_estimator')

        self.target_pose_publisher = rospy.Publisher('target_pose', Pose2D)
        self.pointcloud_subscriber = rospy.Subscriber('pointcloud', PointCloud2, self.pointcloud_updated)
        self.tf_listener = tf.TransformListener()
        self.point_pub = rospy.Publisher('/target_stamped', PointStamped)

    def pointcloud_updated(self, pointcloud):  # type: (PointCloud2) -> None
        average = np.zeros((3,))
        num_points = 0

        min_corner = np.array([-.2, -.3, -10])
        max_corner = np.array([.2, .5, 1.2])

        for point in point_cloud2.read_points(pointcloud, skip_nans=True):
            point = np.array(point[1:])  # type: np.ndarray
            if np.all((point > min_corner) & (point < max_corner)):
                average += point
                num_points += 1
        if num_points == 0:
            rospy.loginfo('No points in pointcloud')
            return
        average = average / num_points

        out_point = PointStamped()
        out_point.header.frame_id = 'camera_depth_frame'
        # out_point.point.x = average[2]
        # out_point.point.y = -average[0]
        # out_point.point.z = -average[2]
        out_point.point.x = average[0]
        out_point.point.y = average[1]
        out_point.point.z = average[2]
        print(out_point)

        out_point = self.tf_listener.transformPoint(target_frame='odom', ps=out_point)

        self.point_pub.publish(out_point)

        target_pose = Pose2D()
        target_pose.x = out_point.point.x
        target_pose.y = out_point.point.y
        self.target_pose_publisher.publish(target_pose)


if __name__ == '__main__':
    e = TargetPositionEstimator()
    rospy.spin()
