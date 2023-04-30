#!/usr/bin/python

# ROS
import rospy
import argparse

# Detect face
from robot_skills import get_robot
from robot_smach_states.human_interaction import FindPeople


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test if the robot can detect the face of the person standing in front"
                                                 "of it")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node('test_detect_faces')
    robot = get_robot(args.robot)

    detect_state = FindPeople(robot=robot,
                              #properties=None,
                              #query_entity_designator=None,
                              #found_people_designator=None,
                              #look_distance=10.0,
                              #speak=False,
                              #strict=True,
                              #nearest=False,
                              attempts=1,
                              #search_timeout=60,
                              look_range=(0.0, 0.0),
                              look_steps=1)
    detect_state.execute(None)
