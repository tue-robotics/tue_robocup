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

class StoreRobocupArena(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot
        robot.base.local_planner.cancelCurrentPlan()

    def execute(self, userdata=None):
        self._robot.ed.update_entity(id="robocup_arena", posestamped=self._robot.base.get_location(), type="waypoint")

        return "done"

class HeadStraight(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot

    def execute(self, userdata=None):
        self._robot.head.look_at_standing_person()
        return "done"

class HeadCancel(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot

    def execute(self, userdata=None):
        self._robot.head.close()
        return "done"

class WaitForOperatorCommand(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["follow", "command"])
        self._robot = robot

    def _confirm(self, tries=3):
        for i in range(0, tries):
            result = self._robot.ears.recognize("<choice>", {"choice" : ["yes", "no"]})
            if 'choice' in result.choices:
                return result.choices['choice'] == "yes"

            if i < tries - 1:
                self._robot.speech.speak("Please say yes or no")

        return False

    def execute(self, userdata=None):
        # Stop the base
        self._robot.base.local_planner.cancelCurrentPlan()

        self._robot.head.look_at_standing_person()

        self._robot.speech.speak("Should I guide you back?")
        if self._confirm():
            self._robot.head.cancel_goal()
            return "command"

        self._robot.head.cancel_goal()
        return "follow"

def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['done', 'aborted'])

    with sm:
        smach.StateMachine.add('INITIALIZE', states.Initialize(robot), transitions={   'initialized':'STORE_ROBOCUP_ARENA', 'abort':'aborted'})
        smach.StateMachine.add('STORE_ROBOCUP_ARENA', StoreRobocupArena(robot), transitions={   'done':'HEAD_STRAIGHT'})
        smach.StateMachine.add('HEAD_STRAIGHT', HeadStraight(robot), transitions={   'done':'SAY_INTRO'})

        smach.StateMachine.add('SAY_INTRO', states.Say(robot, "Hi, Guide me out of the arena please.", look_at_standing_person=True), transitions={ 'spoken' :'FOLLOW_INITIAL'})

        smach.StateMachine.add('FOLLOW_INITIAL', states.FollowOperator(robot, operator_timeout=30,replan=True), transitions={ 'stopped':'WAIT_FOR_OPERATOR_COMMAND', 'lost_operator':'WAIT_FOR_OPERATOR_COMMAND', 'no_operator':'FOLLOW_INITIAL'})

        smach.StateMachine.add('FOLLOW', states.FollowOperator(robot, operator_timeout=30, ask_follow=False, learn_face=False, replan=True), transitions={ 'stopped':'WAIT_FOR_OPERATOR_COMMAND', 'lost_operator':'SAY_GUIDE', 'no_operator':'SAY_GUIDE'})
        smach.StateMachine.add('WAIT_FOR_OPERATOR_COMMAND', WaitForOperatorCommand(robot), transitions={ 'follow':'FOLLOW', 'command':'SAY_GUIDE' })

        smach.StateMachine.add('SAY_GUIDE', states.Say(robot, "I will guide you back to the robocup arena!", look_at_standing_person=True), transitions={ 'spoken' :'GUIDE_TO_ROBOCUP_ARENA'})

        smach.StateMachine.add('GUIDE_TO_ROBOCUP_ARENA', states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="robocup_arena"), radius = knowledge.back_radius),
                                transitions={'arrived': 'SAY_BACK', 'unreachable':'GUIDE_TO_ROBOCUP_ARENA_BACKUP', 'goal_not_defined':'GUIDE_TO_ROBOCUP_ARENA_BACKUP'})

        smach.StateMachine.add('GUIDE_TO_ROBOCUP_ARENA_BACKUP', states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="robocup_arena"), radius = knowledge.back_radius+0.1),
                                transitions={'arrived': 'SAY_BACK', 'unreachable':'GUIDE_TO_ROBOCUP_ARENA', 'goal_not_defined':'GUIDE_TO_ROBOCUP_ARENA'})

        smach.StateMachine.add('SAY_BACK', states.Say(robot, "We are back in the robocup arena!", look_at_standing_person=True), transitions={ 'spoken' :'done'})

        return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('challenge_following_and_guiding')

    startup(setup_statemachine, challenge_name="challenge_following_and_guiding")
