#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D, PointStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import tf
import ros_numpy
from util.cluster import euclidian_clusters, uniform_downsample

import rospy
import numpy as np
from time import sleep

rospy.init_node('navgoals_node')


pose_pub = rospy.Publisher('/target_pose', Pose2D, queue_size=1)
point_pub = rospy.Publisher('/target_stamped', PointStamped, queue_size=1)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    t = rospy.get_time()
    point = PointStamped()
    point.header.frame_id = 'odom'
    point.point.x = 2.0 * np.sin(t/30.0)
    point.point.y = 1.0 * np.cos(t/30.0)

    pose = Pose2D()
    pose.x = point.point.x
    pose.y = point.point.y
    point_pub.publish(point)
    pose_pub.publish(pose)
    sleep(0.01)
print('finished')

