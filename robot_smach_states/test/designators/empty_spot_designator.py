#! /usr/bin/env python

import rospy
import robot_smach_states.util.designators as ds

if __name__ == "__main__":
    rospy.init_node("testdesignator")
    from robot_skills.amigo import Amigo
    robot = Amigo()

    furniture_designator = ds.EdEntityDesignator(robot, id="dinner_table")
    esd = ds.EmptySpotDesignator(robot=robot,
                                 place_location_designator=furniture_designator,
                                 name=None,
                                 area="on_top_of")
    esd.resolve()
