#! /usr/bin/python

import os
import cfgparser
import sys
import rospy

from robocup_knowledge import load_knowledge
from command_recognizer import CommandRecognizer

challenge_knowledge = load_knowledge('challenge_gpsr')

# ----------------------------------------------------------------------------------------------------

def main():
    rospy.init_node("gpsr")

    if len(sys.argv) < 2:
        print "Please specify a robot name 'amigo / sergio'"
        return 1

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        print "unknown robot"
        return 1

    robot = Robot()

    command_recognizer = CommandRecognizer(os.path.dirname(sys.argv[0]) + "/grammar.fcfg", challenge_knowledge)

    robot.head.look_at_standing_person()

    while not rospy.is_shutdown():
        print "---------------------------------------------------------------"

        res = command_recognizer.recognize(robot)

        if res:
            (sentence, semantics) = res
            print "Sentence: %s" % sentence
            print "Semantics: %s" % semantics
        else:
            print "Could not understand"            

# ----------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
