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

from robocup_knowledge import load_knowledge

import action_server
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

    challenge_knowledge = load_knowledge('challenge_eegpsr')

    command_center.set_grammar(os.path.dirname(sys.argv[0]) + "/grammar.fcfg", challenge_knowledge)

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
            hey_robot_wait_forever(robot)

            semantics = command_center.request_command(ask_confirmation=True, ask_missing_info=False)

            if not command_center.execute_command(semantics):
                robot.speech.speak("I am truly sorry, let's try this again")                

            if args.once:
                break

            nwc = NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.starting_pose), radius = 0.3)
            nwc.execute()

# ------------------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
    