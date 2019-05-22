#! /usr/bin/python

import os
import sys
import rospy

from robocup_knowledge import load_knowledge
from robot_skills import get_robot_from_argv

import action_server
from action_server.command_center import CommandCenter

# ----------------------------------------------------------------------------------------------------

def usage():
    print """Usage:

    test_recognition <robot-name>

        (uses speech-to-text, and parses sentence)

or

    test_recognition <robot-name> put your command here

        (parses given command)

or

    test_recognition --from-file <filename>

        (feeds all sentences in <filename> and checks if they can be parsed)
"""

# ----------------------------------------------------------------------------------------------------

def init_command_center(robot=None):
    command_center = CommandCenter(robot)
    challenge_knowledge = load_knowledge('challenge_eegpsr')
    command_center.set_grammar(os.path.dirname(sys.argv[0]) + "/grammar.fcfg", challenge_knowledge)
    return command_center

# ----------------------------------------------------------------------------------------------------

def file_mode(filename):

    command_center = init_command_center()

    with open(filename) as f:
        for sentence in f:
            sentence = sentence.strip()
            res = command_center.parse_command(sentence)
            if not res:
                print 'Could not understand: "{}"'.format(sentence)

# ----------------------------------------------------------------------------------------------------

def sentence_mode(sentence):

    command_center = init_command_center()

    semantics = command_center.parse_command(sentence)

    if semantics:
        print "---------------------------------------------------------------"
        print "semantics: {}".format(semantics)
        print "---------------------------------------------------------------"
    else:
        print 'Could not understand: "{}"'.format(sentence)

# ----------------------------------------------------------------------------------------------------

def speech_mode(robot):

    robot.head.look_at_standing_person()

    command_center = init_command_center(robot)

    while not rospy.is_shutdown():
        # robot.speech.speak("What can I do for you?")

        res = command_center.request_command(ask_confirmation=False, ask_missing_info=False, timeout=600)

        if res:
            (words, semantics) = res

            sentence = " ".join(words)

            print "---------------------------------------------------------------"
            print 'Sentence:  "%s"' % sentence
            print 'Semantics: %s' % semantics
            print "---------------------------------------------------------------"

            robot.speech.speak("You want me to %s" % sentence.replace(" your", " my").replace(" me", " you"), block=True)
        else:
            robot.speech.speak("I did not understand")

# ----------------------------------------------------------------------------------------------------

def main():

    if len(sys.argv) < 2:
        usage()
        return

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if sys.argv[1] == "--from-file":
        file_mode(sys.argv[2])
        return

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    sentence = " ".join([word for word in sys.argv[2:] if word[0] != '_'])

    if sentence:
        sentence_mode(sentence)
    else:

        rospy.init_node("gpsr_test_recognition")

        robot = get_robot_from_argv(index=1)

        speech_mode(robot)

# ----------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
