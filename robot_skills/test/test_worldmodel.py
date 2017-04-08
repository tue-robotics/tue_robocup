#! /usr/bin/env python

import sys
import rospy

import robot_skills.util.kdl_conversions as kdl_conversions

from robot_skills.util.robot_constructor import robot_constructor

if len(sys.argv) < 2:
    print "Please specify a robot name"
    sys.exit()

robot_name = sys.argv[1]

rospy.init_node("worldmodel_test")

robot = robot_constructor(robot_name)

failed_actions = []

#####################
# Test Get Entities #
#####################

all_entities = robot.ed.get_entities()
robot.speech.speak("There are {count} entities in my world".format(count=len(all_entities)))

robot_loc = robot.base.get_location()  # This is just Frame, TODO make this return a FrameStamped
here_vs = kdl_conversions.FrameStamped(robot_loc, "/map").extractVectorStamped()
radius = 2.0
close_entities = robot.ed.get_entities(center_point=here_vs, radius=radius)  # Get all the entities within radius of the robot
robot.speech.speak("There are {count} entities within {radius} meters of my base".format(count=len(close_entities), radius=radius))

if not len(close_entities) < len(all_entities):
    robot.speech.speak("Oops, there should be less close entities than there are entities in the world")
    failed_actions += ["get_entities with center_point and radius"]

typ = "trashbin"
radius_2 = 10.0
close_entities_of_type = robot.ed.get_entities(type=typ, center_point=here_vs, radius=radius_2)  # Get all the entities of some type within radius of the robot
robot.speech.speak("There are {count} {typ}s within {radius} meters of my base".format(count=len(close_entities_of_type), radius=radius_2, typ=typ))

