#!/usr/bin/python

import rospy
import smach
import sys
import math
import time

from visualization_msgs.msg import Marker

import robot_smach_states as states
from robot_smach_states.util.startup import startup

from robot_smach_states.util.designators import EdEntityDesignator, EntityByIdDesignator, analyse_designators
from robot_skills.util import transformations, msg_constructors

from robocup_knowledge import load_knowledge
knowledge = load_knowledge("challenge_following_and_guiding")

import sys, select, termios, tty

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
        smach.State.__init__(self, outcomes=["done"])

        self.settings = termios.tcgetattr(sys.stdin)

    def _getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def execute(self, userdata):
        self._getKey()

        return "done"

def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['done', 'aborted']) 

    with sm:
        smach.StateMachine.add(
            'INITIALIZE', 
            states.Initialize(robot), 
            transitions={   'initialized':'STORE_STARTING_POINT', 
                            'abort':'aborted'})

        smach.StateMachine.add(
            'STORE_STARTING_POINT', 
            StoreStartingPoint(robot), 
            transitions={   'done':'WAIT_FOR_KEY_PRESS1'})

        smach.StateMachine.add(
            'WAIT_FOR_KEY_PRESS1', 
            WaitForKeyPress(), 
            transitions={ 'done': 'SAY_ALSJEBLIEFT'})

        smach.StateMachine.add(
            'SAY_ALSJEBLIEFT', 
            states.Say(robot, "Alsjeblieft Rick. De enveloppen.", look_at_standing_person=True, language='Dutch', block=False), 
            transitions={ 'spoken' :'WAIT_FOR_KEY_PRESS2'})

        smach.StateMachine.add(
            'WAIT_FOR_KEY_PRESS2', 
            WaitForKeyPress(), 
            transitions={ 'done': 'SAY_ALTIJD'})

        smach.StateMachine.add(
            'SAY_ALTIJD', 
            states.Say(robot, "Voor jou altijd, Rick.", look_at_standing_person=True, language='Dutch', block=False), 
            transitions={ 'spoken' :'WAIT_FOR_KEY_PRESS3'})

        smach.StateMachine.add(
            'WAIT_FOR_KEY_PRESS3', 
            WaitForKeyPress(), 
            transitions={ 'done': 'SAY_STEKKER'})

        smach.StateMachine.add(
            'SAY_STEKKER', 
            states.Say(robot, "Ik heb stiekem gekeken, Rick. Maar als ik dat verklap, trekken ze de stekker eruit!", look_at_standing_person=True, language='Dutch', block=False), 
            transitions={ 'spoken' :'WAIT_FOR_KEY_PRESS4'})

        smach.StateMachine.add(
            'WAIT_FOR_KEY_PRESS4', 
            WaitForKeyPress(), 
            transitions={ 'done': 'RESET_HEAD'})

        smach.StateMachine.add(
            'RESET_HEAD', 
            HeadCancel(), 
            transitions={ 'done': 'GO_BACK_TO_STARTING_POINT'})

        smach.StateMachine.add(
            'GO_BACK_TO_STARTING_POINT', 
            states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="starting_point"), radius = knowledge.back_radius, speak=False),
            transitions={   'arrived': 'done', 
                            'unreachable':'GO_BACK_TO_STARTING_POINT', 
                            'goal_not_defined':'aborted'})

        return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('challenge_following_and_guiding')

    startup(setup_statemachine, challenge_name="challenge_following_and_guiding")
