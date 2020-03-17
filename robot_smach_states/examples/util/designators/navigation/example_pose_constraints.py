import rospy
import argparse

# Robot Smach States
from robot_smach_states.util.designators.navigation.pose_constraints import PoseConstraintsDesignator

if __name__ == "__main__":
    """
    Example demonstrating how to use the PoseConstraintsDesignator
    """

    parser = argparse.ArgumentParser(description="Example pose constaints designator")
    args = parser.parse_args()

    rospy.init_node("example_pose_constaints_designator")

    # create the constraints designator
    nav1 = PoseConstraintsDesignator(0.5, 1.3, rz=None, radius=0.15, frame_id="/map", name="pose designator")

    # with orientation
    nav2 = PoseConstraintsDesignator(0.5, 1.3, rz=1.57, radius=0.15, frame_id="/map", name="pose designator orientation")

    rospy.loginfo("Result of {}: {}".format(nav1.name, nav1.resolve()))
    rospy.loginfo("Result of {}: {}".format(nav2.name, nav2.resolve()))
