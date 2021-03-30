#! /usr/bin/python

import random
import sys

import rospy

from hmi import TimeoutException
from robocup_knowledge import load_knowledge

knowledge = load_knowledge('challenge_gpsr')


def main():
    rospy.init_node("test_gpsr_recognition", anonymous=True)
    random.seed()
    if "sergio" in str(sys.argv):
        from robot_skills.sergio import Sergio as Robot
    else:
        from robot_skills.amigo import Amigo as Robot

    robot = Robot()

    while True:
        description = random.choice(["Ask me a question", "Ask me something", "What do you want?", "At your service"])
        robot.speech.speak(description)
        try:
            sentence, semantics = robot.hmi.query(description, knowledge.grammar, knowledge.grammar_target)
        except TimeoutException as e:
            robot.speech.speak("I did not hear anything, please try again")
        else:
            robot.speech.speak("I heard {}".format(sentence))


if __name__ == "__main__":
    sys.exit(main())
