#!/usr/bin/python

import rospy
import smach
import sys
import math
import time
import signal

from visualization_msgs.msg import Marker

import robot_smach_states as states
from robot_smach_states.util.startup import startup

from robot_smach_states.util.designators import EdEntityDesignator, EntityByIdDesignator, analyse_designators
from robot_skills.util import transformations, msg_constructors

from robocup_knowledge import load_knowledge
knowledge = load_knowledge("challenge_following_and_guiding")

import sys, select, termios, tty

class SetPlateCarryingPose(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=["done"])
        self._robot = robot
        
    def execute(self, userdata):
        self._robot.rightArm._send_joint_trajectory([[0,.4,0.5,1.3,.1,0.6,-0.6]])
        self._robot.leftArm._send_joint_trajectory([[-1.2,-.1,.6,2.2,1.0,0.9,0]])
        return 'done'

class StoreStartingPoint(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot
        robot.base.local_planner.cancelCurrentPlan()

    def execute(self, userdata):
        self._robot.ed.update_entity(id="starting_point", posestamped=self._robot.base.get_location(), type="waypoint")

        return "done"

class HeadStraight(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot

    def execute(self, userdata):
        self._robot.head.look_at_standing_person()
        return "done"

class HeadCancel(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot

    def execute(self, userdata):
        self._robot.head.close()
        return "done"

class WaitForKeyPress(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["1", "2", "3", "4", "5"])
        self.settings = termios.tcgetattr(sys.stdin)

    def _getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        return key

    def execute(self, userdata):
        
        while True:
            print "Press one of the following keys:"
            print '\t1:\tzeg "Ik kom eraan"'
            print '\t2:\tzeg "Aljeblieft Rick"'
            print '\t3:\tzeg "Voor jou altijd"'
            print '\t4:\tzeg "Ik heb gespiekt"'
            print '\t5:\tzeg "Oeps!"'

            key = self._getKey()

            if key == '1':
                return "1"
            elif key == '2':
                return "2"
            elif key == '3':
                return "3"
            elif key == '4':
                return "4"
            elif key == '5':
                return "5"
            elif key == "\03":
                sys.exit(0)
            else:
                print "You pressed %s." % key
                print "Don't do that"

def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['done', 'aborted']) 

    with sm:
        smach.StateMachine.add(
            'INITIALIZE', 
            states.Initialize(robot), 
            transitions={   'initialized':'SET_ARM_POSITIONS', 
                            'abort':'aborted'})

        smach.StateMachine.add(
            'SET_ARM_POSITIONS',
            SetPlateCarryingPose(robot),
            transitions={   'done':'WAIT_FOR_KEY_PRESS'})

        smach.StateMachine.add(
            'WAIT_FOR_KEY_PRESS', 
            WaitForKeyPress(), 
            transitions={   '1': 'SAY_IK_KOM',
                            '2': 'SAY_ALSJEBLIEFT',
                            '3': 'SAY_ALTIJD',
                            '4': 'SAY_STEKKER',
                            '5': 'SAY_OEPS'})

        smach.StateMachine.add(
            'SAY_IK_KOM', 
            states.Say(robot, "Ik kom eraan!", look_at_standing_person=True, language='nl', voice='marjolijn', block=False), 
            transitions={ 'spoken' :'WAIT_FOR_KEY_PRESS'})

        smach.StateMachine.add(
            'SAY_ALSJEBLIEFT', 
            states.Say(robot, "Alsjeblieft Rick. De enveloppen.", look_at_standing_person=True, language='nl', voice='marjolijn', block=False), 
            transitions={ 'spoken' :'WAIT_FOR_KEY_PRESS'})

        smach.StateMachine.add(
            'SAY_ALTIJD', 
            states.Say(robot, "Voor jou altijd, Rick.", look_at_standing_person=True, language='nl', voice='marjolijn', block=False), 
            transitions={ 'spoken' :'WAIT_FOR_KEY_PRESS'})

        smach.StateMachine.add(
            'SAY_STEKKER', 
            states.Say(robot, "Ik heb stiekem gekeken, Rick. Maar als ik dat verklap, trekken ze de stekker eruit!", look_at_standing_person=True, language='nl', voice='marjolijn', block=False), 
            transitions={ 'spoken' :'WAIT_FOR_KEY_PRESS'})

        smach.StateMachine.add(
            'RESET_HEAD', 
            HeadCancel(robot), 
            transitions={ 'done': 'GO_BACK_TO_STARTING_POINT'})

        smach.StateMachine.add(
            'GO_BACK_TO_STARTING_POINT', 
            states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="starting_point"), radius = knowledge.back_radius, speak=False),
            transitions={   'arrived': 'done', 
                            'unreachable':'WAIT_FOR_KEY_PRESS', 
                            'goal_not_defined':'aborted'})

        smach.StateMachine.add(
            'SAY_OEPS', 
            states.Say(robot, "Oeps!", look_at_standing_person=True, language='nl', voice='marjolijn', block=False), 
            transitions={ 'spoken' :'WAIT_FOR_KEY_PRESS'})

        return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('challenge_following_and_guiding')

    startup(setup_statemachine, challenge_name="challenge_following_and_guiding")
