#! /usr/bin/env python

from __future__ import print_function

import sys

import rospy

import PyKDL as kdl
from pykdl_ros import VectorStamped

from robot_skills import get_robot

if len(sys.argv) < 2:
    print("Please specify a robot name")
    sys.exit()

robot_name = sys.argv[1]

rospy.init_node("worldmodel_test")

robot = get_robot(robot_name)

failed_actions = []

#####################
# Test Get Entities #
#####################

all_entities = robot.ed.get_entities()
robot.speech.speak("There are {count} entities in my world".format(count=len(all_entities)))

robot_loc = VectorStamped.from_framestamped(robot.base.get_location())
radius = 2
close_entities = robot.ed.get_entities(center_point=robot_loc, radius=radius)  # Get all the entities within radius of the robot
robot.speech.speak("There are {count} entities within {radius} meters of my base"
                   .format(count=len(close_entities), radius=radius))

if not len(close_entities) < len(all_entities):
    robot.speech.speak("Oops, there should be less close entities than there are entities in the world")
    failed_actions += ["get_entities with center_point and radius"]

# Get all the entities of some type within radius of the robot
robot_loc = VectorStamped.from_framestamped(robot.base.get_location())
query_type = "trashbin"
radius_2 = 10
close_entities_of_type = robot.ed.get_entities(etype=query_type, center_point=robot_loc, radius=radius_2)
robot.speech.speak("There are {count} {type}s within {radius} meters of my base"
                   .format(count=len(close_entities_of_type), radius=radius_2, type=query_type))

#######################
# Test Closest Entity #
#######################

# TODO: center_point should also be a VectorStamped
closest = robot.ed.get_closest_entity(center_point=VectorStamped(kdl.Vector(), rospy.Time.now(), "map"), radius=2.0)
robot.speech.speak("The closest entity to the center of the arena is {uuid}, of type {etype}"
                   .format(uuid=closest.uuid[:10], etype=closest.etype))

closest2 = robot.ed.get_closest_entity(etype=query_type,
                                       center_point=VectorStamped(kdl.Vector(), rospy.Time.now(), "map"),
                                       radius=10.0)
robot.speech.speak("The closest {etype} to the center of the arena is {uuid}".format(uuid=closest2.uuid, etype=query_type))
if closest2.etype != query_type:
    failed_actions += ["get_closest_entity with type, center_point and radius"]


#######################
# Test Get Entity #
#######################

uuid = "trashbin"
trashbin = robot.ed.get_entity(uuid)
robot.speech.speak("Entity {uuid} is at {x}, {y} in the map".format(uuid=uuid, x=trashbin.pose.frame.p.x(),
                                                                    y=trashbin.pose.frame.p.y()))
if trashbin.uuid != uuid:
    failed_actions += ["get_entity with id {uuid}".format(uuid=uuid)]
    robot.speech.speak("I could not get entity with id {uuid}".format(uuid=uuid))
