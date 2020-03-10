#! /usr/bin/env python

from __future__ import print_function

import rospy
import argparse
from robot_smach_states.manipulation.place import EmptySpotDesignator
import robot_smach_states.util.designators as ds
from robot_skills import get_robot

if __name__ == "__main__":
    rospy.init_node("testdesignator")
    parser = argparse.ArgumentParser(description="Test the empty spot designator")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    robot = get_robot(args.robot)

    furniture_designator = ds.EntityByIdDesignator(robot, id="dinner_table")
    arm_designator = ds.UnoccupiedArmDesignator(robot, {})

    def with_area():
        esd = EmptySpotDesignator(robot=robot,
                                  place_location_designator=furniture_designator,
                                  arm_designator=arm_designator,
                                  name="with_area",
                                  area="on_top_of")
        print(esd.resolve())

    def without_area():
        esd = EmptySpotDesignator(robot=robot,
                                  place_location_designator=furniture_designator,
                                  arm_designator=arm_designator,
                                  name="without_area")
        print(esd.resolve())

    with_area()
    without_area()
