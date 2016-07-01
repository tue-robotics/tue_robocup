#! /usr/bin/python

# ------------------------------------------------------------------------------------------------------------------------
# By Sjoerd van den Dries, 2016
# ------------------------------------------------------------------------------------------------------------------------

import os
import sys
import rospy
import argparse
import time

import std_msgs

import robot_smach_states
from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint, NavigateToSymbolic

from robocup_knowledge import load_knowledge

import action_server
from action_server.command_center import CommandCenter

# ------------------------------------------------------------------------------------------------------------------------

class ChallengeOpen:

    # ------------------------------------------------------------------------------------------------------------------------

    def __init__(self, robot, sentence=None):
        self.robot = robot
        self._trigger_sub = rospy.Subscriber("/" + robot.robot_name + "/trigger", std_msgs.msg.String, self._trigger_callback, queue_size=1)

        self.do_listen_command = False
        self.sentence = sentence

    # ------------------------------------------------------------------------------------------------------------------------

    def _trigger_callback(self, msg):
        print "trigger"
        if msg.data == "listen" or msg.data == "gpsr":
            print "listen"
            self.do_listen_command = True
            self.sentence = None
        else:
            self.do_listen_command = False
            self.sentence = msg.data

    # ------------------------------------------------------------------------------------------------------------------------

    def run(self):

        command_center = CommandCenter(self.robot)

        challenge_knowledge = load_knowledge('challenge_open')

        command_center.set_grammar(os.path.dirname(sys.argv[0]) + "/grammar.fcfg", challenge_knowledge)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        sentences = [
                "Hey hello there! What can I do for you?"
                "I'm so sorry! Please tell me what I can do for you!"
                ]

        rospy.loginfo("Ready, waiting for trigger!")

        while not rospy.is_shutdown():

            command_semantics = None

            if self.sentence:
                command_semantics = command_center.parse_command(self.sentence)
                if not command_semantics:
                    self.robot.speech.speak("I cannot parse \"{}\"".format(self.sentence))
                self.sentence = None

            elif self.do_listen_command:
                command_semantics = command_center.request_command(ask_confirmation=True, ask_missing_info=False,
                                                                   sentences=sentences, n_tries=2)
                if not command_semantics:
                    self.robot.speech.speak("I did not understand")
                self.do_listen_command = False
            else:
                time.sleep(0.1)

            if command_semantics:
                print "Command semantics: {}".format(command_semantics)
                command_center.execute_command(command_semantics)
        
# ------------------------------------------------------------------------------------------------------------------------

def main():
    rospy.init_node("challenge_open")

    parser = argparse.ArgumentParser()
    parser.add_argument('robot', help='Robot name')
    parser.add_argument('sentence', nargs='*', help='Optional sentence')
    args = parser.parse_args()
    rospy.loginfo('args: %s', args)

    sentence = " ".join([word for word in args.sentence if word[0] != '_'])

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

    challenge = ChallengeOpen(robot, sentence)
    challenge.run()

if __name__ == "__main__":
    sys.exit(main())
    