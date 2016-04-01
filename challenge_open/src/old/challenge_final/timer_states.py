#!/usr/bin/python

import rospy
import smach
import robot_smach_states as states

from challenge_open.srv import *

class StartPresentationTimer(smach.State):
    def __init__(self, robot, mins=0, secs=0):
        smach.State.__init__(self, outcomes=['done','failed'])
        if mins==0 and secs==0:
            rospy.logerr('StartPresentationTimer: Time was not set')
        else:
            self.mins = mins
            self.secs = secs
        self.robot = robot


    def execute(self, userdata):
        try:
            rospy.wait_for_service('/amigo/timer/start_countdown',1.0)
            start_countdown = rospy.ServiceProxy('/amigo/timer/start_countdown', StartCountdown)
        except rospy.ServiceException as e:
            print 'Timer service is not available: ' + str(e)
            return 'failed'

        try:
            if start_countdown(mins=self.mins, secs=self.secs):
                print 'Set clock succesfully!'
                return 'done'
            else:
                return 'Strangely, setting the clock was not successful'
                return 'failed'

        except rospy.ServiceException as e:
            print 'Setting timer failed: ' + str(e) 
            return 'failed'

class SayRemainingTime(smach.State):
    def __init__(self, robot, block=False):
        smach.State.__init__(self, outcomes=['done','failed'])
        self.robot = robot
        self.block = block

    def execute(self,userdata):
        try:
            rospy.wait_for_service('/amigo/timer/get_time',1.0)
            get_time = rospy.ServiceProxy('/amigo/timer/get_time', EmptyString)
        except rospy.ServiceException as e:
            print 'Timer service is not available: ' + str(e)
            return 'failed'

        try:
            response = get_time()
            self.robot.speech.speak(sentence=str(response.sentence), block=self.block)
            print 'Said time succesfully!'
            return 'done'
        except rospy.ServiceException as e:
            print 'Timer service failed: ' + str(e)
            return 'failed'
