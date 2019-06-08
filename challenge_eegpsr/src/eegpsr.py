#! /usr/bin/python

# ------------------------------------------------------------------------------------------------------------------------
# By Sjoerd van den Dries, 2016
# ------------------------------------------------------------------------------------------------------------------------

import os
import sys
import rospy
import argparse
import time

from robot_skills import get_robot
from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint, NavigateToSymbolic
from robot_smach_states.util.designators import EntityByIdDesignator
from robocup_knowledge import load_knowledge
from robot_smach_states import StartChallengeRobust

from action_server.command_center import CommandCenter


# ------------------------------------------------------------------------------------------------------------------------

def hey_robot(robot):

    robot.head.look_at_standing_person()
    robot.head.wait_for_motion_done()

    spec = "hey %s" % robot.robot_name

    res = robot.ears.recognize(spec=spec, time_out=rospy.Duration(60))

    if not res:
        robot.speech.speak("My ears are not working properly, sorry!")
        return False

    return res.result == spec

# ------------------------------------------------------------------------------------------------------------------------

def hey_robot_wait_forever( robot):
    while not hey_robot(robot):
        pass

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

    robot = get_robot(args.robot)

    # Sleep for 1 second to make sure everything is connected
    time.sleep(1)

    command_center = CommandCenter(robot)

    challenge_knowledge = load_knowledge('challenge_eegpsr')

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

    first = True

    if sentence:
        semantics = command_center.parse_command(sentence)
        if not semantics:
            rospy.logerr("Cannot parse \"{}\"".format(sentence))
            return

        command_center.execute_command(semantics)
    else:

        while True:

            if first:
                # First sentence robot says
                sentences = ["Hello there! Welcome to the double E GPSR. You can give me an order, but wait for the ping."]
            else:
                # Sentence robot says after completing a task
                sentences = ["Hello there, you look lovely! I'm here to take a new order, but wait for the ping!"] 

            # These sentences are for when the first try fails
            # (Robot says "Do you want me to ...?", you say "No", then robot says sentence below)
            sentences += [
                "I'm so sorry! Can you please speak louder and slower? And wait for the ping!",
                "Again, I am deeply sorry. Bad robot! Please try again, but wait for the ping!",
                "You and I have communication issues. Speak up! Tell me what you want, but wait for the ping"
                ]

            hey_robot_wait_forever(robot)                

            res = command_center.request_command(ask_confirmation=True, ask_missing_info=False, timeout=600, sentences=sentences)           

            if not res:
                continue

            first = False

            (sentence, semantics) = res

            if not command_center.execute_command(semantics):
                robot.speech.speak("I am truly sorry, let's try this again")

            if args.once:
                break

            if not args.skip:
                nwc = NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.starting_pose), radius = 0.3)
                nwc.execute()

# ------------------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
