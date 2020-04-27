import rospy
import argparse

from robot_skills.get_robot import get_robot
# Robot Smach States
from robot_smach_states.navigation.constraint_functions.symbolic_constraints import symbolic_constraint, room_constraint
from robot_smach_states.util.designators.ed_designators import EntityByIdDesignator

if __name__ == "__main__":
    """
    Example demonstrating how to use the symbolic_constraint() function
    Make sure you have both (robot)-start and ED running.
    """

    parser = argparse.ArgumentParser(description="Example symbolic_constaint() function")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("example_symbolic_constraints")
    robot = get_robot(args.robot)

    # get an entitydesignator
    entity = EntityByIdDesignator(robot, 'dinner_table', name='entity_designator')

    # in front of the dinner table
    rospy.loginfo("symbolic constraint corresponding to 'in_front_of' the 'dinner_table':")
    pc, oc = symbolic_constraint(robot, {entity: "in_front_of"})
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))

    # get a roomdesignator
    room = EntityByIdDesignator(robot, 'kitchen', name='room_designator')

    # room constraint
    rospy.loginfo("room constraint corresponding to 'kitchen':")
    pc, oc = room_constraint(robot, room)
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))
