import rospy
import argparse

from robot_skills.get_robot import get_robot
# Robot Smach States
from robot_smach_states.navigation.constraint_functions.radius_constraints import radius_constraint
from robot_smach_states.util.designators.ed_designators import EntityByIdDesignator

if __name__ == "__main__":
    """
    Example demonstrating how to use the radius_constraint() function
    Make sure you have both (robot)-start and ED running.
    """

    parser = argparse.ArgumentParser(description="Example radius_constaints function()")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("example_radius_constaints")
    robot = get_robot(args.robot)

    # get an entitydesignator
    entity = EntityByIdDesignator(robot, 'dinner_table', name='entity_designator')

    # base function
    rospy.loginfo("Stay 0.5 meter away from the dinner table with a margin of 10cm")
    rospy.loginfo("radius_constraint(entity, radius=0.5, margin=0.1)")
    pc, oc = radius_constraint(entity, radius=0.5, margin=0.1)
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))
