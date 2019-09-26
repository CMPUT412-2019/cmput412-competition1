#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D, PointStamped
from sensor_msgs.msg import PointCloud2, LaserScan
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
        # self.pointcloud_subscriber = rospy.Subscriber('pointcloud', PointCloud2, self.pointcloud_updated)
        self.tf_listener = tf.TransformListener()
        self.point_pub = rospy.Publisher('/target_stamped', PointStamped)
        self.pose = None  # type: Pose2D
        self.pose_subscriber = rospy.Subscriber('/pose2d', Pose2D, self.pose_callback)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def pose_callback(self, message):
        self.pose = message

    def __scan_callback(self, scan):  # type: (LaserScan) -> None
        if self.pose is None:
            rospy.loginfo('Waiting for pose...')
            return
        scan = scan  # type: LaserScan
        
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, num=len(scan.ranges), endpoint=True)
        angle_mid = (scan.angle_min + scan.angle_max) / 2.0
        ranges[angles <= (angle_mid - 0.25)] = np.nan

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
        self.point_pub.publish(out_point)
        out_point = self.tf_listener.transformPoint(target_frame='/odom', ps=out_point)
        target_pose = Pose2D()
        target_pose.x = out_point.point.x
        target_pose.y = out_point.point.y
        self.target_pose_publisher.publish(target_pose)

    def scan_callback(self, scan):  # type: (LaserScan) -> None
        if self.pose is None:
            rospy.loginfo('Waiting for pose...')
            return
        scan = scan  # type: LaserScan

        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, num=len(scan.ranges), endpoint=True)
        angle_mid = (scan.angle_min + scan.angle_max) / 2.0

        ranges = ranges[angles <= (angle_mid - 0.25)]
        angles = angles[angles <= (angle_mid - 0.25)]

        x = ranges * np.sin(angles)
        y = ranges * np.cos(angles)
        z = np.zeros_like(x)
        xyz = np.column_stack((x, y, z))
        clusters = euclidian_clusters(xyz, 0.2)
        cluster_xs = [np.mean(x[clusters == i], axis=0)
                      for i in np.unique(clusters)]
        cluster_sizes = [np.sum(clusters == i) for i in np.unique(clusters)]
        try:
            index = np.nanargmin(cluster_sizes)
        except ValueError:
            print("no points found")
            return

        min_range = ranges[index]
        min_angle = angles[index]
        # Convert to a point
        out_point = PointStamped()
        out_point.header.frame_id = scan.header.frame_id
        out_point.point.x = min_range * np.cos(min_angle)
        out_point.point.y = min_range * np.sin(min_angle)
        self.point_pub.publish(out_point)
        out_point = self.tf_listener.transformPoint(target_frame='/odom', ps=out_point)
        target_pose = Pose2D()
        target_pose.x = out_point.point.x
        target_pose.y = out_point.point.y
        self.target_pose_publisher.publish(target_pose)

    def pointcloud_updated(self, pointcloud):  # type: (PointCloud2) -> None
        if self.pose is None:
            rospy.loginfo('Waiting for pose...')
            return
        min_corner = np.array([-1, -10, 0])
        max_corner = np.array([1, 0.2, 2])

        points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pointcloud)
        bound_mask = np.all((points > min_corner) & (points < max_corner), axis=1)
        points = points[bound_mask, :]

        if points.shape[0] == 0:
            rospy.loginfo('No points in pointcloud')
            target = None
        else:
            target = np.mean(points, axis=0)

        # points = uniform_downsample(points, 0.01)
        # clusters = euclidian_clusters(points, 0.1)
        # targets = [np.mean(points[clusters == i], axis=0)
        #            for i in np.unique(clusters)]
        # distances = np.linalg.norm(targets)
        # target = targets[np.argmin(distances)]

        out_point = PointStamped()
        out_point.header.frame_id = pointcloud.header.frame_id
        out_point.point.x = 0 if target is None else target[0]
        out_point.point.y = 0 if target is None else target[1]
        out_point.point.z = 0 if target is None else target[2]
        # print(out_point)

        out_point = self.tf_listener.transformPoint(target_frame='/odom', ps=out_point)

        dx = self.pose.x - out_point.point.x
        dy = self.pose.y - out_point.point.y
        out_point.point.x += dx / np.sqrt(dx**2 + dy**2) * 1.0
        out_point.point.y += dy / np.sqrt(dx**2 + dy**2) * 1.0

        self.point_pub.publish(out_point)

        if target is not None:
            target_pose = Pose2D()
            target_pose.x = out_point.point.x
            target_pose.y = out_point.point.y
            self.target_pose_publisher.publish(target_pose)


if __name__ == '__main__':
    e = TargetPositionEstimator()
    rospy.spin()
