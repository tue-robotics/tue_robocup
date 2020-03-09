#!/usr/bin/python
from __future__ import absolute_import
import rospy
import argparse
import PyKDL as kdl

# TU/e Robotics
import robot_smach_states.util.designators as ds
from robot_skills.get_robot import get_robot
from robot_skills.util.kdl_conversions import FrameStamped

from robocup_knowledge import load_knowledge

# Challenge storing groceries
from challenge_storing_groceries import NearObjectSpotDesignator
challenge_knowledge = load_knowledge('challenge_storing_groceries')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--robot", default="hero", help="Name of the robot you want to use")
    args = parser.parse_args()

    rospy.init_node('test_inspect_shelves')

    robot = get_robot(args.robot)

    # add entity on the table
    entity_id = "test_item"
    pose = FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(3.1, 2.7, 0.7)),
                        frame_id="/map")
    robot.ed.update_entity(id=entity_id, frame_stamped=pose)

    shelfDes = ds.EntityByIdDesignator(robot, id=challenge_knowledge.shelf)
    itemDes = ds.EntityByIdDesignator(robot, entity_id)
    des = NearObjectSpotDesignator(robot, itemDes, shelfDes)
    print des.resolve()
