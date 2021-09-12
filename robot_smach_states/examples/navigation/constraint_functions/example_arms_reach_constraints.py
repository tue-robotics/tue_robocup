import rospy
import argparse

import PyKDL as kdl
from pykdl_ros import FrameStamped

from robot_skills.get_robot import get_robot
from robot_skills.util.entity import Entity

# Robot Smach States
from robot_smach_states.navigation.constraint_functions.arms_reach_constraints import arms_reach_constraint
from robot_smach_states.util.designators.arm import ArmDesignator
from robot_smach_states.util.designators.core import Designator
from robot_smach_states.util.designators.utility import AttrDesignator


if __name__ == "__main__":
    """
    Example demonstrating how to use the arms_reach_constraint function to generate navigation constraints
    Make sure you have a (robot)-start running since a robot object is created for this.
    """

    parser = argparse.ArgumentParser(description="Example arms_reach_constaint() function")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("example_arms_reach_constraints")
    robot = get_robot(args.robot)

    # get an armdesignator and a pose designator
    arm_designator = ArmDesignator(robot, {}, name="arm_designator")
    pose = FrameStamped(kdl.Frame(kdl.Vector(0, 0, 1.3)), rospy.Time.now(), "map")
    pose_designator = Designator(pose, name="frame designator")

    # basic functionality
    rospy.loginfo("Example of the basic funtionality")
    rospy.loginfo("Running: pc, oc = arms_reach_constraint(pose_designator, arm_designator)")
    pc, oc = arms_reach_constraint(pose_designator, arm_designator)
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))

    # dont look at the frame
    rospy.loginfo("Example where we don't require the robot to look at the pose")
    rospy.loginfo("Running: pc, oc = arms_reach_constraint(pose_designator, arm_designator, look=False)")
    pc, oc = arms_reach_constraint(pose_designator, arm_designator, look=False)
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))

    # Use an entity
    entity = Entity("dummy_entity", "dummy_type", "map", pose.frame, None, None, None, None)
    entity_designator = Designator(entity)

    rospy.loginfo("When you have an entity rather than a pose, consider wrapping it in an attribute designator")
    rospy.loginfo("Running: pc, oc = arms_reach_constraint(AttrDesignator(entity_designator, 'pose', resolve_type=FrameStamped), arm_designator)")
    pc, oc = arms_reach_constraint(AttrDesignator(entity_designator, 'pose', resolve_type=FrameStamped), arm_designator)
    rospy.loginfo("pc becomes : {}".format(pc))
    rospy.loginfo("oc becomes : {}".format(oc))
