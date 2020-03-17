import rospy
import argparse

from robot_skills.get_robot import get_robot
from robot_skills.util.kdl_conversions import frame_stamped
# Robot Smach States
from robot_smach_states.util.designators.navigation.arm_constraints import ArmsreachConstraintsDesignator
from robot_smach_states.util.designators.arm import ArmDesignator
from robot_smach_states.util.designators.core import VariableDesignator

if __name__ == "__main__":
    """
    Example demonstrating how to use the ArmsreachConstraintsDesignator
    Make sure you have a (robot)-start running since a robot object is created for this.
    """

    parser = argparse.ArgumentParser(description="Example arm constaints designator")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("example_armsreach_constraint_designator")
    robot = get_robot(args.robot)

    # get an armdesignator, framedesignator and
    arm = ArmDesignator(robot, {}, name="arm_designator")
    frame = frame_stamped("/map", 0, 0, 1.3)
    framedes = VariableDesignator(frame, name="frame designator")

    # create an armsreach constraintdesignator
    # look at the frame
    nav1 = ArmsreachConstraintsDesignator(robot, framedes, arm, name="armsreach designator")

    # dont look at the frame
    nav2 = ArmsreachConstraintsDesignator(robot, framedes, arm, look=False, name="armsreach designator no look")

    # resolve the designators
    rospy.loginfo("Result of {}: {}".format(nav1.name, nav1.resolve()))
    rospy.loginfo("Result of {}: {}".format(nav2.name, nav2.resolve()))
