import rospy
import smach
import robot_smach_states as states

from challenge_final.srv import *

class StartPresentationTimer(smach.State):
    def __init__(self, robot, block=False):
        State.__init__(self, outcomes=['done','failed'])
        self.robot = robot

    def execute(self, userdata):
        try:
            rospy.wait_for_service('finals/start_countdown',1.0)
            set_clock = rospy.ServiceProxy('finals/start_countdown', StartCountdown)
        except rospy.ServiceException as e:
            print 'Timer service is not available: ' + str(e)
            return 'failed'

        try:
            if set_clock(mins=10, secs=0):
                return 'done'
            else:
                return 'failed'

        except rospy.ServiceException as e:
            print 'Setting timer failed: ' + str(e) 
            return 'failed'

class SayRemainingTime(smach.State):
    def __init__(self, robot, block=False):
        State.__init__(self, outcomes=['done','failed'])
        self.robot = robot
        self.block = block

    def execute(self,userdata):
        try:
            rospy.wait_for_service('finals/get_time',1.0)
            get_time = rospy.ServiceProxy('finals/get_time', EmptyString)
        except rospy.ServiceException as e:
            print 'Timer service is not available: ' + str(e)
            return 'failed'

        try:
            sentence = get_time(mins=10, secs=0)
            robot.speech.speak(sentence, self.block)
            return 'done'
        except rospy.ServiceException as e:
            print 'Timer service failed: ' + str(e)
            return 'failed'
