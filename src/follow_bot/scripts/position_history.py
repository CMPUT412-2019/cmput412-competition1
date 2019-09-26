#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D, PointStamped, PolygonStamped, Point32
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import tf
import ros_numpy
from util.cluster import euclidian_clusters, uniform_downsample

import rospy
import numpy as np
from time import sleep
from collections import deque

rospy.init_node('position_history_node')

history = deque()
max_length = 10

history_publisher = rospy.Publisher('target_history', PolygonStamped, queue_size=1)

def on_target_update(target_pose):  # type: (Pose2D) -> None
    if len(history) == max_length:
        history.popleft()
    history.append(target_pose)
    p = PolygonStamped()
    p.header.frame_id = 'odom'
    p.polygon.points = []
    for pose in history:
        pose = pose  # type: Pose2D
        point = Point32()
        point.x = pose.x
        point.y = pose.y
        point.z = 0.1
        p.polygon.points.append(point)
    history_publisher.publish(p)


target_sub = rospy.Subscriber('target_pose', Pose2D, on_target_update, queue_size=1)
rospy.spin()
