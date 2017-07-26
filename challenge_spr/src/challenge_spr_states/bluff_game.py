#!/usr/bin/python
import threading

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

# def _turn_to_closest_entity(robot):
#     # Reset the world model just to be sure
#     robot.ed.reset()
#
#     operator = None
#     while not operator:
#         operator = robot.ed.get_closest_entity(radius=1.9, center_point=robot.base.get_location().extractVectorStamped())
#         print operator
#         if not operator:
#             vth = 0.5
#             th = 3.1415 / 10
#             print "Turning %f radians with force drive" % th
#             robot.base.force_drive(0, 0, vth, th / vth)
#
#     robot.base.force_drive(0, 0, 0, 0.5)
#
#     # Turn towards the operator
#     current = robot.base.get_location()
#     robot_th = current.frame.M.GetRPY()[2]  # Get the Yaw, rotation around Z
#     desired_th = math.atan2(operator._pose.p.y() - current.frame.p.y(),
#                             operator._pose.p.x() - current.frame.p.x())
#
#     # Calculate params
#     th = desired_th - robot_th
#     if th > 3.1415:
#         th -= 2 * 3.1415
#     if th < -3.1415:
#         th += 2 * 3.1415
#     vth = 0.5
#
#     # Turn
#     robot.base.force_drive(0, 0, (th / abs(th)) * vth, abs(th) / vth)


def turn_to_closest_entity(robot):

    rospy.sleep(1.5)

    start = rospy.Time.now()
    yaw = None

    # Try to find a source for a couple of seconds
    while not yaw and (rospy.Time.now() - start).to_sec() < 6:
        yaw = robot.ssl.get_last_yaw(.1)
        rospy.sleep(0.05)

    # If we did not find a yaw, just default
    if yaw is None:
        yaw = math.pi * 2 / 5

    if yaw > math.pi:
        yaw -= 2 * math.pi
    if yaw < -math.pi:
        yaw += 2 * math.pi

    rospy.loginfo("I should turn %.2f rad now", yaw)

    vyaw = 0.5
    robot.base.force_drive(0, 0, (yaw / abs(yaw)) * vyaw, abs(yaw) / vyaw)


class HearQuestion(smach.State):
    def __init__(self, robot, time_out=15.0):
        smach.State.__init__(self, outcomes=["answered", "not_answered"],input_keys=['crowd_data'])
        self.robot = robot
        self.time_out = time_out

    def execute(self, userdata):
        crowd_data = userdata.crowd_data

        self.robot.head.look_at_standing_person()

        t = threading.Thread(target=turn_to_closest_entity, args=(self.robot,))
        t.start()

        res = hear(self.robot, time_out=self.time_out)

        t.join()

        return answer(self.robot, res, crowd_data)


class HearQuestionRepeat(smach.State):
    def __init__(self, robot, time_out=15.0):
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
            "males": 1,
            "men": 2,
            "females": 3,
            "women": 4,
            "children": 5,
            "boys": 6,
            "girls": 7,
            "adults": 8,
            "elders": 9,
            "crowd_size": 10
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
