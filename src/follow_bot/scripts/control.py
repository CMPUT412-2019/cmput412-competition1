#!/usr/bin/env python

import math
import numpy as np

import rospy
from smach import State, StateMachine
from smach_ros import IntrospectionServer

from state.follow import FollowState


rospy.init_node('control_node')

sm = StateMachine(outcomes=['ok', 'err'])
with sm:
    StateMachine.add('FOLLOW', FollowState(), transitions={'ok': 'err'})

sis = IntrospectionServer('smach_server', sm, '/SM_ROOT')
sis.start()

sm.execute()
