import rospy
import argparse

from robot_skills.util.entity import Entity
import PyKDL as kdl
# Robot Smach States
from robot_smach_states.navigation.constraint_functions.waypoint_constraints import waypoint_constraint
from robot_smach_states.util.designators.core import Designator

if __name__ == "__main__":
    """
    Example demonstrating how to use the waypoint_constraint() function
    """

    parser = argparse.ArgumentParser(description="Example waypoint_constraint() function")
    args = parser.parse_args()

    rospy.init_node("example_waypoint_constaints")

    # get an entitydesignator
    pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(1, 2, 0))
    e = Entity("dummy_waypoint", "dummy_type", "/map", pose, None, None, None, None)
    waypoint= Designator(e, name="entity designator")

    # waypoint example
    rospy.loginfo("waypoint_constraint resolves to:")
    pc, oc = waypoint_constraint(waypoint, radius=1.0)
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))
