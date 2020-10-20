import rospy
import argparse

# Robot Smach States
from robot_smach_states.navigation.constraint_functions.compound_constraints import combine_constraints
from robot_smach_states.navigation.constraint_functions.pose_constraints import pose_constraints

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Example combine_constraints function")
    args = parser.parse_args()
    rospy.init_node("example_combine_constraints")

    rospy.loginfo("Example combine constraints function")

    # function 1
    rospy.loginfo("First constraint function resolves to:")
    fun1 = lambda: pose_constraints(0, 3.0, radius=3.0)
    pc, oc = fun1()
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))

    # function 2
    rospy.loginfo("Second constraint function resolves to:")
    fun2 = lambda: pose_constraints(0, 0.0, 1.57, radius=1.0)
    pc, oc = fun2()
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))

    # combined constraints
    rospy.loginfo("The combined constraints become:")
    pc, oc = combine_constraints([fun1, fun2])
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))
