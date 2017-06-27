#! /usr/bin/env python

import sys
import rospy

from robot_skills.util.robot_constructor import robot_constructor

if len(sys.argv) < 2:
    print "Please specify a robot name"
    sys.exit()

robot_name = sys.argv[1]

rospy.init_node("test_person_recognition")

robot = robot_constructor(robot_name)

try:
    OPERATOR_NAME = sys.argv[2]
except IndexError:
    OPERATOR_NAME = "operator"

SAMPLES = 5
MAX_ATTEMPTS = 15


robot.head.look_at_standing_person()
robot.speech.speak("{name}, please stand in front of me".format(name=OPERATOR_NAME))
rospy.sleep(1)

total_learn_attempts, successful_learn_attempts = 0, 0
while total_learn_attempts < MAX_ATTEMPTS and successful_learn_attempts < SAMPLES:
    total_learn_attempts += 1
    if robot.head.learn_person(OPERATOR_NAME):
        successful_learn_attempts += 1

if successful_learn_attempts:
    robot.speech.speak("I learned {name} after {count} attempts".format(name=OPERATOR_NAME, count=total_learn_attempts))
else:
    robot.speech.speak("Learning {name} failed after {count} attempts. Exit test".format(name=OPERATOR_NAME, count=total_learn_attempts))

    exit(-1)

robot.speech.speak("To test, someone step in front of me.")
rospy.sleep(5)

total_recognize_attempts, successful_recognize_attempts = 0, 0
while total_recognize_attempts < MAX_ATTEMPTS and successful_recognize_attempts < 5:
    total_recognize_attempts += 1

    # raw_detections is a list of Recognitions
    # a recognition contains a CategoricalDistribution
    # a CategoricalDistribution is a list of CategoryProbabilities
    # a CategoryProbability has a label and a float
    raw_detections = robot.head.detect_faces()

    if raw_detections:
        robot.speech.speak("There are {count} raw_recognitions".format(count=len(raw_detections)))

        best_detection = robot.head.get_best_face_recognition(raw_detections, OPERATOR_NAME)
        if best_detection:
            robot.speech.speak("The best detection has probability {prob} for label {lbl}".format(prob=best_detection.probability, lbl=best_detection.label))
            successful_recognize_attempts += 1
        else:
            robot.speech.speak("{name} not detected".format(name=OPERATOR_NAME))
    else:
        robot.speech.speak("No raw detections")

    rospy.sleep(1)
