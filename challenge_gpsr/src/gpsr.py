#! /usr/bin/python

# ------------------------------------------------------------------------------------------------------------------------
# By Rokus Ottervanger, 2017
# ------------------------------------------------------------------------------------------------------------------------

import sys
import rospy
import argparse
import time
import random

import actionlib
import action_server.msg

import hmi

from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint, NavigateToSymbolic
from robot_smach_states import StartChallengeRobust
from robot_smach_states.util.designators import EntityByIdDesignator

from robocup_knowledge import load_knowledge


class ActionClient(object):
    def __init__(self, robot_name):
        self._robot_name = robot_name
        action_name = self._robot_name + "/action_server/task"
        self._action_client = actionlib.SimpleActionClient(action_name, action_server.msg.TaskAction)
        rospy.loginfo("Waiting for task action server to come online...")
        self._action_client.wait_for_server()
        rospy.loginfo("Connected to task action server")

    def send_task(self, semantics):
        """
        Send a task to the action server.

        A task is composed of one or multiple actions.
        :param semantics: A json string with a list of dicts, every dict in the list has at least an 'action' field,
        and depending on the type of action, several parameter fields may be required.
        :return: True or false, and a message specifying the outcome of the task
        """
        recipe = semantics

        goal = action_server.msg.TaskGoal(recipe=recipe)
        self._action_client.send_goal(goal)
        self._action_client.wait_for_result()
        result = self._action_client.get_result()

        msg = ""
        if result.result == action_server.msg.TaskResult.RESULT_MISSING_INFORMATION:
            return False, "Not enough information to perform this task."
        elif result.result == action_server.msg.TaskResult.RESULT_TASK_EXECUTION_FAILED:
            return False, "Task execution failed."
        elif result.result == action_server.msg.TaskResult.RESULT_UNKNOWN:
            return False, "Unknown result from the action server."
        elif result.result == action_server.msg.TaskResult.RESULT_SUCCEEDED:
            return True, "Task succeeded!"

        return False, msg

# ------------------------------------------------------------------------------------------------------------------------

def main():
    rospy.init_node("gpsr")
    random.seed()

    # parser = argparse.ArgumentParser()
    # parser.add_argument('robot', help='Robot name')
    # parser.add_argument('--once', action='store_true', help='Turn off infinite loop')
    # parser.add_argument('--skip', action='store_true', help='Skip enter/exit')
    # # parser.add_argument('sentence', nargs='*', help='Optional sentence')
    # args = parser.parse_args()
    # rospy.loginfo('args: %s', args)

    skip        = rospy.get_param('~skip', False)
    once        = rospy.get_param('~once', False)
    robot_name  = rospy.get_param('~robot_name')
    rospy.loginfo("Parameters:")
    rospy.loginfo("robot_name = {}".format(robot_name))
    if skip:
        rospy.loginfo("skip = {}".format(skip))
    if once:
        rospy.loginfo("once = {}".format(once))

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        raise ValueError('unknown robot')

    robot = Robot()

    # Sleep for 1 second to make sure everything is connected
    # time.sleep(1)

    action_client = ActionClient(robot.robot_name)

    knowledge = load_knowledge('challenge_gpsr')


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # Start

    if not skip:

        # Wait for door, enter arena
        s = StartChallengeRobust(robot, knowledge.initial_pose)
        s.execute()

        # Move to the start location
        nwc = NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=knowledge.starting_pose), radius = 0.3)
        nwc.execute()

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    while True:

        user_instruction = "What can I do for you?"

        try:
            res = robot.hmi.query(knowledge.grammar, knowledge.grammar_target)
        except hmi.TimeOutException:
            robot.speech.speak(random.sample(knowledge.not_understood_sentences, 1))
            continue

        (sentence, semantics) = res

        task_result = action_client.send_task(semantics)

        if not task_result:
            robot.speech.speak("I am truly sorry, let's try this again")
            continue

        if once:
            break

        if not skip:
            nwc = NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=knowledge.starting_pose), radius = 0.3)
            nwc.execute()

# ------------------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
