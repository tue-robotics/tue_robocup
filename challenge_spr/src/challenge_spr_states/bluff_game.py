#!/usr/bin/python

import roslib
import rospy
import smach
import sys
import random
import math
import time

from robot_skills.util import transformations as tf
from robot_smach_states.util.startup import startup
from robot_skills.util import transformations, msg_constructors

from robot_smach_states import Initialize, Say
from robot_smach_states.util.designators import Designator, EdEntityDesignator

from robocup_knowledge import load_knowledge

from riddle_game import hear, answer



def _turn_to_closest_entity(robot):
    # Reset the world model just to be sure
    robot.ed.reset()

    operator = None
    while not operator:
        operator = robot.ed.get_closest_entity(radius=1.9, center_point=robot.base.get_location().p)
        print operator
        if not operator:
            vth = 0.5
            th = 3.1415 / 10
            print "Turning %f radians with force drive" % th
            robot.base.force_drive(0, 0, vth, th / vth)

    robot.base.force_drive(0, 0, 0, 0.5)

    # Turn towards the operator
    current = robot.base.get_location()
    robot_th = current.M.GetRPY()[2]  # Get the Yaw, rotation around Z
    desired_th = math.atan2(operator._pose.p.y() - current.p.y(),
                            operator._pose.p.x() - current.p.x())

    # Calculate params
    th = desired_th - robot_th
    if th > 3.1415:
        th -= 2 * 3.1415
    if th < -3.1415:
        th += 2 * 3.1415
    vth = 0.5

    # Turn
    robot.base.force_drive(0, 0, (th / abs(th)) * vth, abs(th) / vth)

def turn_to_closest_entity(robot):

    print "Last talker id: " + robot.hmi.last_talker_id

    # Calculate params
    if "dragonfly_speech_recognition" not in robot.hmi.last_talker_id:
        # TUrn
        vth = 0.5
        th = 3.1415
        robot.base.force_drive(0, 0, (th / abs(th)) * vth, abs(th) / vth)

    _turn_to_closest_entity(robot)


class HearQuestion(smach.State):
    def __init__(self, robot, time_out=rospy.Duration(15)):
        smach.State.__init__(self, outcomes=["answered", "not_answered"],input_keys=['crowd_data'])
        self.robot = robot
        self.time_out = time_out

    def execute(self, userdata):
        crowd_data = userdata.crowd_data

        self.robot.head.look_at_standing_person()

        res = hear(self.robot, time_out=self.time_out)

        turn_to_closest_entity(self.robot)

        return answer(self.robot, res, crowd_data)


class HearQuestionRepeat(smach.State):
    def __init__(self, robot, time_out=rospy.Duration(15)):
        smach.State.__init__(self, outcomes=["answered", "not_answered"], input_keys=['crowd_data'])
        self.robot = robot
        self.time_out = time_out

    def execute(self, userdata):
        crowd_data = userdata.crowd_data

        self.robot.head.look_at_standing_person()

        res = hear(self.robot, time_out=self.time_out)

        return answer(self.robot, res, crowd_data)



# Standalone testing -----------------------------------------------------------------


class TestBluffGame(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        self.userdata.crowd_data = {
            'males': 2,
            'females': 3
        }

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   Initialize(robot),
                                   transitions={'initialized': 'BLUFF_GAME_1',
                                                'abort': 'Aborted'})

            smach.StateMachine.add('BLUFF_GAME_1',
                                   HearQuestion(robot),
                                   transitions={'answered': 'Done',
                                                'not_answered': 'BLUFF_GAME_1_ASK_REPEAT'},
                                   remapping={'crowd_data':'crowd_data'})


            smach.StateMachine.add("BLUFF_GAME_1_ASK_REPEAT",
                                   Say(robot, "Could you please repeat your question?"),
                                   transitions={"spoken": "BLUFF_GAME_1_REPEAT"})

            smach.StateMachine.add('BLUFF_GAME_1_REPEAT',
                                   HearQuestionRepeat(robot),
                                   transitions={'answered' :'Done',
                                                'not_answered': 'Done'},
                                   remapping={'crowd_data':'crowd_data'})


if __name__ == "__main__":
    rospy.init_node('speech_person_recognition_exec')

    startup(TestBluffGame, challenge_name="challenge_spr")
