#!/usr/bin/env python
import rospy
from robocup_knowledge import load_knowledge
from robot_smach_states.util.startup import startup
from robot_smach_states.util.designators import EdEntityByIdDesignator

from empty_shelf_designator import EmptyShelfDesignator

challenge_knowledge = load_knowledge('challenge_storing_groceries')

USE_SLAM = True  # Indicates whether or not to use SLAM for localization
if USE_SLAM:
    CABINET = challenge_knowledge.cabinet_slam
else:
    CABINET = challenge_knowledge.cabinet_amcl

PLACE_SHELF = challenge_knowledge.place_shelf


def setup_statemachine(robot):
    cabinet = EdEntityByIdDesignator(robot, id="bookcase", name="pick_shelf")

    ds = EmptyShelfDesignator(robot, cabinet, name="placement", area=PLACE_SHELF)

    for i in range(5):
        ds.resolve()
        import ipdb;ipdb.set_trace()


if __name__ == '__main__':
    rospy.init_node('test_empty_shelf_designator')

    startup(setup_statemachine)
