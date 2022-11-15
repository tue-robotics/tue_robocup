import os
import rospy
import argparse
import std_srvs.srv
import robot_smach_states.util.designators as ds

from robot_skills.get_robot import get_robot
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToSymbolic


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test the NavigateToSymbolic state")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    parser.add_argument("entity", help="Entity name of the object to navigate to, for example dinner_table")
    parser.add_argument("area", default="near", help="area of the entity to navigate to, for example in_front_of")
    args = parser.parse_args()

    # Create node, robot and toggle interface
    rospy.init_node("test_navigate_to_symbolic")
    r = get_robot(args.robot)
    e_id = args.entity
    e_area = args.area

    # Instantiate GuideToSymbolic machine
    s = NavigateToSymbolic(r,
                           {ds.EntityByIdDesignator(r, uuid=e_id): e_area},
                           ds.EntityByIdDesignator(r, uuid=e_id)
                           )
    # Execute the state
    s.execute()
