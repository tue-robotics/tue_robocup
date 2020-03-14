import rospy
import argparse

# Robot Smach States
from robot_smach_states.util.designators.navigation.navigation import CompoundConstraintsDesignator
from robot_smach_states.util.designators.navigation.pose_constraints import PoseConstraintsDesignator

from collections import namedtuple


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test the atttrDesignator")
    args = parser.parse_args()
    rospy.init_node("test_attrDesignator")

    # Create the compound designator
    comp = CompoundConstraintsDesignator()

    # create multiple navigationconstraint designators
    nav1 = PoseConstraintsDesignator(x=1,
                                     y=2,
                                     rz=1.57,
                                     radius=1.3
                                     )

    nav2 = PoseConstraintsDesignator(x=1,
                                     y=3,
                                     radius=1
                                     )

    # add the designators to the compound designator
    comp.add(nav1, 'first')
    comp.add(nav2, 'second')

    # resolve the designators
    rospy.loginfo("Result of nav1: {}".format(nav1.resolve()))
    rospy.loginfo("Result of nav2: {}".format(nav2.resolve()))
    rospy.loginfo("Result of comp: {}".format(comp.resolve()))
