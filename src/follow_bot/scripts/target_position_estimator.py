#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D, PointStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import tf
import ros_numpy
from util.cluster import euclidian_clusters, uniform_downsample


class TargetPositionEstimator:
    def __init__(self):
        rospy.init_node('target_position_estimator')

        self.target_pose_publisher = rospy.Publisher('target_pose', Pose2D)
        self.bounded_ptcloud_publisher = rospy.Publisher('bounded_pointcloud', PointCloud2)
        self.pointcloud_subscriber = rospy.Subscriber('pointcloud', PointCloud2, self.pointcloud_updated)
        self.tf_listener = tf.TransformListener()
        self.point_pub = rospy.Publisher('/target_stamped', PointStamped)

    def pointcloud_updated(self, pointcloud):  # type: (PointCloud2) -> None
        min_corner = np.array([-1, -10, 0])
        max_corner = np.array([1, 0.2, 2])

        points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pointcloud)
        bound_mask = np.all((points > min_corner) & (points < max_corner), axis=1)
        points = points[bound_mask, :]

        if points.shape[0] == 0:
            rospy.loginfo('No points in pointcloud')
            return

        points = uniform_downsample(points, 0.2)
        clusters = euclidian_clusters(points, 0.4)
        targets = [np.mean(points[clusters == i], axis=0)
                   for i in np.unique(clusters)]
        distances = np.linalg.norm(targets)
        target = targets[np.argmin(distances)]

        out_point = PointStamped()
        out_point.header.frame_id = pointcloud.header.frame_id
        out_point.point.x = target[0]
        out_point.point.y = target[1]
        out_point.point.z = target[2]
        print(out_point)

        out_point = self.tf_listener.transformPoint(target_frame='/odom', ps=out_point)

        self.point_pub.publish(out_point)

        target_pose = Pose2D()
        target_pose.x = out_point.point.x
        target_pose.y = out_point.point.y
        self.target_pose_publisher.publish(target_pose)


if __name__ == '__main__':
    e = TargetPositionEstimator()
    rospy.spin()
