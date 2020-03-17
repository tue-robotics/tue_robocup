import rospy
import argparse

from robot_skills.util.entity import Entity
import PyKDL as kdl
# Robot Smach States
from robot_smach_states.util.designators.navigation.look_at_constraints import LookAtConstraintsDesignator
from robot_smach_states.util.designators.core import VariableDesignator

if __name__ == "__main__":
    """
    Example demonstrating how to use the LookAtConstraintsDesignator
    """

    parser = argparse.ArgumentParser(description="Example look at constaints designator")
    args = parser.parse_args()

    rospy.init_node("example_look_at_constaints_designator")

    # get an entitydesignator
    pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(1, 2, 1.3))
    e = Entity("dummy", "dummy_type", "/map", pose, None, None, None, None)
    entity = VariableDesignator(e, name="entity designator")

    # create the constraints designator
    nav1 = LookAtConstraintsDesignator(entity, name="look at designator")

    rospy.loginfo("Result of {}: {}".format(nav1.name, nav1.resolve()))
