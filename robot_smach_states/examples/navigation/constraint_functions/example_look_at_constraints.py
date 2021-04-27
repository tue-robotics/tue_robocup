import rospy
import argparse

from robot_skills.util.entity import Entity
import PyKDL as kdl
# Robot Smach States
from robot_smach_states.navigation.constraint_functions.look_at_constraints import look_at_constraint
from robot_smach_states.util.designators.core import Designator

if __name__ == "__main__":
    """
    Example demonstrating how to use the look_at_constraints function
    """

    parser = argparse.ArgumentParser(description="Example look_at_constaints function")
    args = parser.parse_args()

    rospy.init_node("example_look_at_constaints")

    # get an entitydesignator
    pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(1, 2, 1.3))
    e = Entity("dummy", "dummy_type", "map", pose, None, None, None, None)
    entity = Designator(e, name="entity designator")

    # example base function
    rospy.loginfo("look at an entity")
    pc, oc = look_at_constraint(entity)
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))

    rospy.loginfo("orient yourself w.r.t an entity")
    pc, oc = look_at_constraint(entity, offset=1.57)  # keep the object 90 degrees to your left
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))
