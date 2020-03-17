import rospy
import argparse

from robot_skills.get_robot import get_robot
# Robot Smach States
from robot_smach_states.util.designators.navigation.symbolic_constraints import SymbolicConstraintsDesignator
from robot_smach_states.util.designators.ed_designators import EntityByIdDesignator

if __name__ == "__main__":
    """
    Example demonstrating how to use the RadiusConstraintsDesignator
    Make sure you have both (robot)-start and ED running.
    """

    parser = argparse.ArgumentParser(description="Example radius constaints designator")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("example_radius_constaints_designator")
    robot = get_robot(args.robot)

    # get an entitydesignator
    entity = EntityByIdDesignator(robot, 'dinner_table', name='entity_designator')

    # create the constraints designator
    nav1 = SymbolicConstraintsDesignator(robot, {entity: "in_front_of"}, name="radius designator")

    rospy.loginfo("Result of {}: {}".format(nav1.name, nav1.resolve()))
