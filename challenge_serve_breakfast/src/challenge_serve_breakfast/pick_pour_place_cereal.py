#
# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import os

import rospy
from pykdl_ros import FrameStamped

from challenge_serve_breakfast.tuning import (
    get_item_pour_poses,
    JOINTS_POST_PICK,
)
from challenge_serve_breakfast.navigate_to_and_place_item_on_table import PlaceItemOnTable
from robot_skills import get_robot
from smach import StateMachine, cb_interface, CBState


class PickPourPlaceCereal(StateMachine):
    def __init__(self, robot, table_id):
        StateMachine.__init__(self, outcomes=["succeeded"])
        # noinspection PyProtectedMember
        arm = robot.get_arm()._arm

        def send_joint_goal(position_array, wait_for_motion_done=True):
            # noinspection PyProtectedMember
            arm._send_joint_trajectory([position_array], timeout=0.0)
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string, max_torque=0.1, wait_for_motion_done=True):
            arm.gripper.send_goal(open_close_string, max_torque=max_torque)

        def send_goal(pose_goal, wait_for_motion_done=True):
            arm.send_goal(pose_goal, timeout=0.0)
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        @cb_interface(outcomes=["done"])
        def _pick(_):
            robot.speech.speak("Lets grab some cereal", block=False)
            send_gripper_goal("close")
            send_joint_goal(JOINTS_POST_PICK)
            return "done"


        @cb_interface(outcomes=["done"])
        def _pour(_):
            pour_target = "bowl"
            item_name = "cereal"
            rospy.loginfo(f"pouring in {pour_target}...")

            robot.speech.speak(f"I am going to pour the {item_name}", block=False)

            robot.head.look_up()
            robot.head.wait_for_motion_done()

            item_pour_poses = get_item_pour_poses(pour_target)
            for pose in item_pour_poses:
                goal_fs = FrameStamped(pose, rospy.Time(0), table_id)
                rospy.loginfo("Pouring...")
                send_goal(goal_fs)
            robot.head.reset()
            rospy.loginfo("Retracting...")
            robot.base.force_drive(-0.1, 0, 0, 3)  # Drive backwards at 0.1m/s for 3s, so 30cm
            arm.send_joint_goal("carrying_pose")
            return "done"

        @cb_interface(outcomes=["done"], output_keys=["item_picked"])
        def _set_userdata(user_data):
            user_data["item_picked"] = "cereal_box"
            return "done"

        with self:
            self.add("PICK", CBState(_pick), transitions={"done": "POUR"})
            self.add("POUR", CBState(_pour), transitions={"done": "SET_USERDATA"})
            self.add("SET_USERDATA", CBState(_set_userdata), transitions={"done": "PLACE"})
            self.add("PLACE", PlaceItemOnTable(robot, table_id), transitions={"succeeded": "succeeded",
                                                                              "failed": "succeeded"})


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    PickPourPlaceCereal(robot_instance, "kitchen_table").execute()
