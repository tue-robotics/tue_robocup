import rospy
import argparse

from robot_skills.get_robot import get_robot
# Robot Smach States
from robot_smach_states.util.designators.navigation.waypoint_constraints import WayPointConstraintsDesignator
from robot_smach_states.util.designators.ed_designators import EntityByIdDesignator

if __name__ == "__main__":
    """
    Example demonstrating how to use the WaypointConstraintsDesignator
    Make sure you have both (robot)-start and ED running.
    """

    parser = argparse.ArgumentParser(description="Example waypoint constaints designator")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("example_symbolic_constaints_designator")
    robot = get_robot(args.robot)

    # get an entitydesignator
    wp = EntityByIdDesignator(robot, 'initial_pose', name='entity_designator')

    # create the constraints designator
    nav1 = WayPointConstraintsDesignator(wp, name="waypoint designator")

    rospy.loginfo("Result of {}: {}".format(nav1.name, nav1.resolve()))
