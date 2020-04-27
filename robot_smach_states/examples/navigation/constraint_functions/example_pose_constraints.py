import rospy
import argparse

# Robot Smach States
from robot_smach_states.navigation.constraint_functions.pose_constraints import pose_constraints

if __name__ == "__main__":
    """
    Example demonstrating how to use the pose_constraints() function
    """

    parser = argparse.ArgumentParser(description="Example pose_constaints() function")
    args = parser.parse_args()

    rospy.init_node("example_pose_constaints")

    # base function
    rospy.loginfo("Pose constraint function with only x and y resolves to:")
    pc, oc = pose_constraints(x=1.0, y=2.0)
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))

    # with rotation
    rospy.loginfo("Pose constraint function with x, y and rz resolves to:")
    pc, oc = pose_constraints(x=1.0, y=2.0, rz=1.57)
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))

    # extra inputs
    rospy.loginfo("You may also specify the radius and the frame_id:")
    pc, oc = pose_constraints(x=1.0, y=2.0, rz=1.57, radius=3.0, frame_id="/example_frame")
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))

