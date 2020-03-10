#ROS
import rospy
import argparse

# TU/e Robotics
from robot_skills.get_robot import get_robot
from robot_skills.arms import GripperState, GripperTypes
# Robot Smach States
from robot_smach_states.manipulation import SetGripper
import robot_smach_states.util.designators as ds


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test the gripper state whether it can open and close")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("Set_gripper_state")
    robot = get_robot(args.robot)
    arm = ds.UnoccupiedArmDesignator(robot, {"required_gripper_types": [GripperTypes.GRASPING]})

    for gripper_state in [GripperState.CLOSE, GripperState.OPEN]:
        set_gripper_state = SetGripper(robot, arm, gripperstate=gripper_state)
        outcome = set_gripper_state.execute()
        rospy.loginfo("Set gripper state to {}, outcome: {}".format(gripper_state, outcome))
        rospy.sleep(2.5)
