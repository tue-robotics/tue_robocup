#! /usr/bin/python

# ------------------------------------------------------------------------------------------------------------------------
# By Sjoerd van den Dries, 2016
# ------------------------------------------------------------------------------------------------------------------------

import os
import sys
import rospy
import argparse
import time

import robot_smach_states
from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint, NavigateToSymbolic
from robot_smach_states import StartChallengeRobust
from robot_smach_states.util.designators import EntityByIdDesignator

from robocup_knowledge import load_knowledge

import action_server
from action_server.command_center import CommandCenter

# ------------------------------------------------------------------------------------------------------------------------

def main():
    rospy.init_node("ee_gpsr")

    parser = argparse.ArgumentParser()
    parser.add_argument('robot', help='Robot name')
    parser.add_argument('--once', action='store_true', help='Turn on infinite loop')
    parser.add_argument('--skip', action='store_true', help='Skip enter/exit')
    parser.add_argument('sentence', nargs='*', help='Optional sentence')
    args = parser.parse_args()
    rospy.loginfo('args: %s', args)

    mock_sentence = " ".join([word for word in args.sentence if word[0] != '_'])

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if args.robot == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif args.robot == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        raise ValueError('unknown robot')

    robot = Robot()

    # Sleep for 1 second to make sure everything is connected
    time.sleep(1)

    command_center = CommandCenter(robot)

    challenge_knowledge = load_knowledge('challenge_gpsr')

    command_center.set_grammar(os.path.dirname(sys.argv[0]) + "/grammar.fcfg", challenge_knowledge)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # Start

    if not args.skip:

        # Wait for door, enter arena
        s = StartChallengeRobust(robot, challenge_knowledge.initial_pose)
        s.execute()

        # Move to the start location
        nwc = NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.starting_pose), radius = 0.3)
        nwc.execute()

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    sentence = " ".join([word for word in args.sentence if word[0] != '_'])

    if sentence:
        semantics = command_center.parse_command(sentence)
        if not semantics:
            rospy.logerr("Cannot parse \"{}\"".format(sentence))
            return

        command_center.execute_command(semantics)
    else:
        while True:
            (sentence, semantics) = command_center.request_command(ask_confirmation=True, ask_missing_info=False)

            if not command_center.execute_command(semantics):
                robot.speech.speak("I am truly sorry, let's try this again")

            if args.once:
                break

            nwc = NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.starting_pose), radius = 0.3)
            nwc.execute()

# ------------------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
