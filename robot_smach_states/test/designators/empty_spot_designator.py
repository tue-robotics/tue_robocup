#! /usr/bin/env python

import rospy
import robot_smach_states.util.designators as ds
from robot_skills import get_robot_from_argv

if __name__ == "__main__":
    rospy.init_node("testdesignator")

    robot = get_robot_from_argv(index=1)

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
