import rospy
import argparse

from robot_skills.util.entity import Entity
import PyKDL as kdl
# Robot Smach States
from robot_smach_states.util.designators.navigation.waypoint_constraints import WayPointConstraintsDesignator
from robot_smach_states.util.designators.core import VariableDesignator

if __name__ == "__main__":
    """
    Example demonstrating how to use the WaypointConstraintsDesignator
    """

    parser = argparse.ArgumentParser(description="Example waypoint constaints designator")
    args = parser.parse_args()

    rospy.init_node("example_waypoint_constaints_designator")

    # get an entitydesignator
    pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(1, 2, 0))
    e = Entity("dummy_waypoint", "dummy_type", "/map", pose, None, None, None, None)
    waypoint= VariableDesignator(e, name="entity designator")

    # create the constraints designator
    nav1 = WayPointConstraintsDesignator(waypoint, name="waypoint designator")

    rospy.loginfo("Result of {}: {}".format(nav1.name, nav1.resolve()))
