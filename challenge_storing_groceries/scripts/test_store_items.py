#!/usr/bin/python

import rospy
import argparse

# TU/e Robotics
import robot_smach_states.util.designators as ds
from robot_skills.get_robot import get_robot
from robot_skills.util.entity import Entity
from robot_smach_states.navigation import Find
from robot_smach_states.manipulation import EmptySpotDesignator

from robocup_knowledge import load_knowledge

# Challenge storing groceries
from challenge_storing_groceries import StoreItems
challenge_knowledge = load_knowledge('challenge_storing_groceries')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--robot", default="hero", help="Name of the robot you want to use")
    args = parser.parse_args()

    rospy.init_node('test_store_multiple_items')

    robot = get_robot(args.robot)

    tableDes = ds.EntityByIdDesignator(robot, id=challenge_knowledge.table)
    ItemDes = ds.VariableDesignator(resolve_type=Entity)
    placeDes = EmptySpotDesignator(robot, tableDes, ds.ArmDesignator(robot, {}))

    s = StoreItems(robot, tableDes, placeDes)
    result = s.execute(None)
    print result
