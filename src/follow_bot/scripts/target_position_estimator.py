#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D, PointStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import tf
import ros_numpy


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

        min_corner = np.array([-1, -10, 0])
        max_corner = np.array([1, 0.2, 10])

        self.min_x = rospy.get_param("~min_x", -0.2)
        self.min_y = rospy.get_param("~min_y", -0.3)

        self.max_x = rospy.get_param("~max_x", 0.2)
        self.max_y = rospy.get_param("~max_y", 0.5)
        self.max_z = rospy.get_param("~max_z", 1.2)

        points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pointcloud)
        bound_mask = np.all((points > min_corner) & (points < max_corner), axis=1)
        points = points[bound_mask, :]

        if points.shape[0] == 0:
            rospy.loginfo('No points in pointcloud')
            return

        average = np.mean(points, axis=0)

        out_point = PointStamped()
        out_point.header.frame_id = pointcloud.header.frame_id
        # out_point.point.x = average[2]
        # out_point.point.y = -average[0]
        # out_point.point.z = -average[2]
        out_point.point.x = average[0]
        out_point.point.y = average[1]
        out_point.point.z = average[2]
        print(out_point)

        # out_point = self.tf_listener.transformPoint(target_frame='/odom', ps=out_point)

        self.point_pub.publish(out_point)

        target_pose = Pose2D()
        target_pose.x = out_point.point.x
        target_pose.y = out_point.point.y
        self.target_pose_publisher.publish(target_pose)


if __name__ == '__main__':
    e = TargetPositionEstimator()
    rospy.spin()
