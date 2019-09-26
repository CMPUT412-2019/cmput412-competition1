#!/usr/bin/env python

import math
import numpy as np

import rospy
from smach import State, StateMachine
from smach_ros import IntrospectionServer

from state.follow import FollowState
from sensor_msgs.msg import Joy

class WaitForJoystick(State):
    def __init__(self):
        State.__init__(self, outcomes=['ok', 'err'])
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.waiting = True
    def execute(self, ud):
        self.waiting = True
        while self.waiting:
            rospy.sleep(0.01)
        return 'ok'
    def joy_callback(self, message):
        self.waiting = False


rospy.init_node('control_node')

sm = StateMachine(outcomes=['ok', 'err'])
with sm:
    StateMachine.add('WAIT', WaitForJoystick(), transitions={'ok': 'FOLLOW'})
    StateMachine.add('FOLLOW', FollowState())

sis = IntrospectionServer('smach_server', sm, '/SM_ROOT')
sis.start()

sm.execute()
