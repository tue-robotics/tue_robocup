#! /usr/bin/env python

import sys
import rospy
import robot_smach_states.util.designators as ds
from robot_skills import get_robot

if __name__ == "__main__":
    rospy.init_node("testdesignator")

    robot_name = sys.argv[1] if len(sys.argv) > 1 else "amigo"
    robot = get_robot(robot_name)

    furniture_designator = ds.EdEntityDesignator(robot, id="dinner_table")

    def with_area():
        esd = ds.EmptySpotDesignator(robot=robot,
                                     place_location_designator=furniture_designator,
                                     name=None,
                                     area="on_top_of")
        print esd.resolve()

    def without_area():
        esd = ds.EmptySpotDesignator(robot=robot,
                                     place_location_designator=furniture_designator,
                                     name=None)
        print esd.resolve()

    with_area()
    without_area()
