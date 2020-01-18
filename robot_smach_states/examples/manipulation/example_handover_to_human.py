# ROS
import rospy
import argparse

# TU/e Robotics
from robot_skills.get_robot import get_robot
from robot_skills import arms

# Robot Smach States
import robot_smach_states.util.designators as ds
from robot_smach_states.manipulation import HandoverFromHuman, HandoverToHuman
from robot_smach_states import LockDesignator


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test handover to human state")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("test_handover_stuff")
    robot = get_robot(args.robot)

    rospy.loginfo("Creating arm designator")
    arm_designator = ds.UnoccupiedArmDesignator(robot=robot,
                                                arm_properties={"required_goals": ["handover_to_human", "reset"],
                                                                "required_gripper_types": [arms.GripperTypes.GRASPING]},
                                                name='arm_des').lockable()

    rospy.loginfo("Creating lock designator state")
    s0 = LockDesignator(arm_designator)

    rospy.loginfo("Creating handover from human state")
    s1 = HandoverFromHuman(robot, arm_designator, grabbed_entity_label="foo")

    rospy.loginfo("Creating handover to human state")
    s2 = HandoverToHuman(robot, arm_designator)

    rospy.loginfo("Executing lock designator state")
    s0.execute()

    rospy.loginfo("Executing handover from human state")
    s1.execute()

    rospy.loginfo("Executing handover to human state")
    s2.execute()

    rospy.loginfo("Done")
