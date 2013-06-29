#!/usr/bin/python

import roslib; roslib.load_manifest('challenge_final')
import rospy

import smach

from robot_skills.reasoner  import Conjunction, Compound

class AssertCurrentOperator(smach.State):
    """Get the object ID of the person in front of the robot and assert it as the current operator"""

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["asserted", "no_operator"])
        self.robot = robot
        self.query_find_person_in_front_of_robot = Conjunction(
                                                    Compound("instance_of",    "ObjectID",   Compound("exact", "person")),
                                                    Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")))

    def execute(self, userdata=None):
        """Queries the reasoner for which persons are in front of the robot, sets the closest one to current_operator"""
        answers = self.robot.reasoner.query(self.query_find_person_in_front_of_robot)

        if answers:
            objectId = answers[0]["ObjectID"]
            self.robot.reasoner.assertz(Compound("current_operator", objectId))
            return "asserted"
        else:
            return "no_operator"
