#! /usr/bin/env python

import sys
import rospy

import robot_skills.util.kdl_conversions as kdl_conversions
import PyKDL as kdl

from robot_skills.util.robot_constructor import robot_constructor

if len(sys.argv) < 2:
    print("Please specify a robot name")
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

robot_loc = robot.base.get_location().extractVectorStamped()
radius = 2
close_entities = robot.ed.get_entities(center_point=robot_loc, radius=radius)  # Get all the entities within radius of the robot
robot.speech.speak("There are {count} entities within {radius} meters of my base"
                   .format(count=len(close_entities), radius=radius))

if not len(close_entities) < len(all_entities):
    robot.speech.speak("Oops, there should be less close entities than there are entities in the world")
    failed_actions += ["get_entities with center_point and radius"]

# Get all the entities of some type within radius of the robot
robot_loc = robot.base.get_location().extractVectorStamped()
query_type = "trashbin"
radius_2 = 10
close_entities_of_type = robot.ed.get_entities(type=query_type, center_point=robot_loc, radius=radius_2)
robot.speech.speak("There are {count} {type}s within {radius} meters of my base"
                   .format(count=len(close_entities_of_type), radius=radius_2, type=query_type))

#######################
# Test Closest Entity #
#######################

# TODO: center_point should also be a VectorStamped
closest = robot.ed.get_closest_entity(center_point=kdl_conversions.VectorStamped(), radius=2.0)  # This is implicitly in /map
robot.speech.speak("The closest entity to the center of the arena is {id}, of type {type}"
                   .format(id=closest.id[:10], type=closest.type))

closest2 = robot.ed.get_closest_entity(type=query_type, center_point=kdl_conversions.VectorStamped(), radius=10.0)
robot.speech.speak("The closest {type} to the center of the arena is {id}".format(id=closest2.id, type=query_type))
if closest2.type != query_type:
    failed_actions += ["get_closest_entity with type, center_point and radius"]


#######################
# Test Get Entity #
#######################

id_ = "trashbin"
trashbin = robot.ed.get_entity(id_)
robot.speech.speak("Entity {id} is at {x}, {y} in the map".format(id=id_, x=trashbin.pose.frame.p.x(), y=trashbin.pose.frame.p.y()))
if trashbin.id != id_:
    failed_actions += ["get_entity with id {id}".format(id=id_)]
    robot.speech.speak("I could not get entity with id {id}".format(id=id_))
