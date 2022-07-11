#
# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import os

import PyKDL
import rospy

from challenge_serve_breakfast.tuning import JOINTS_POST_PICK, ITEM_VECTOR_DICT, item_vector_to_item_frame, \
    item_frame_to_pose, POUR_OFFSET_Y, JOINTS_PRE_POUR, JOINTS_POUR, JOINTS_PLACE_HORIZONTAL, JOINTS_RETRACT, \
    POUR_OFFSET_X
from robot_skills import get_robot
from robot_smach_states.navigation.control_to_pose import ControlToPose, ControlParameters
from smach import StateMachine, cb_interface, CBState


class PickPourPlaceCereal(StateMachine):
    def __init__(self, robot, table_id):
        StateMachine.__init__(self, outcomes=["succeeded"])
        # noinspection PyProtectedMember
        arm = robot.get_arm()._arm

        def send_joint_goal(position_array, wait_for_motion_done=True):
            # noinspection PyProtectedMember
            arm._send_joint_trajectory([position_array], timeout=rospy.Duration(0))
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string, max_torque=0.1, wait_for_motion_done=True):
            arm.gripper.send_goal(open_close_string, max_torque=max_torque)
            if wait_for_motion_done:
                rospy.sleep(1.0)  # Does not work with motion_done apparently

        @cb_interface(outcomes=["done"])
        def _pick(_):
            robot.speech.speak("Lets grab some cereal", block=False)
            send_gripper_goal("close")
            send_joint_goal(JOINTS_POST_PICK)
            return "done"

        @cb_interface(outcomes=["done"])
        def _align_pour(_):
            item_placement_vector = ITEM_VECTOR_DICT["cereal_box"] + PyKDL.Vector(POUR_OFFSET_X, POUR_OFFSET_Y, 0)
            item_frame = item_vector_to_item_frame(item_placement_vector)

            goal_pose = item_frame_to_pose(item_frame, table_id)
            rospy.loginfo("Moving to pouring pose at {}".format(goal_pose))
            robot.head.look_down()
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute({})
            return "done"

        @cb_interface(outcomes=["done"])
        def _pour(_):
            robot.speech.speak("Hope this goes well", block=False)
            send_joint_goal(JOINTS_PRE_POUR)
            send_joint_goal(JOINTS_POUR)
            send_joint_goal(JOINTS_PRE_POUR)
            send_joint_goal(JOINTS_POST_PICK)
            return "done"

        @cb_interface(outcomes=["done"])
        def _align_place(_):
            robot.speech.speak("Awesome", block=False)
            item_placement_vector = ITEM_VECTOR_DICT["cereal_box"]
            item_frame = item_vector_to_item_frame(item_placement_vector)

            goal_pose = item_frame_to_pose(item_frame, table_id)
            rospy.loginfo("Moving to place pose at {}".format(goal_pose))
            robot.head.look_down()
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute({})
            return "done"

        @cb_interface(outcomes=["done"])
        def _place(_):
            robot.speech.speak("Putting back the cereal", block=False)
            rospy.loginfo("Placing...")
            send_joint_goal(JOINTS_PLACE_HORIZONTAL)
            send_gripper_goal("open")
            robot.head.look_up()

            rospy.loginfo("Retract...")
            send_joint_goal(JOINTS_RETRACT, wait_for_motion_done=False)
            robot.base.force_drive(-0.1, 0, 0, 3)  # Drive backwards at 0.1m/s for 3s, so 30cm
            send_gripper_goal("close", wait_for_motion_done=False)
            arm.send_joint_goal("carrying_pose")

            robot.head.reset()

            return "done"

        with self:
            self.add("PICK", CBState(_pick), transitions={"done": "ALIGN_POUR"})
            self.add("ALIGN_POUR", CBState(_align_pour), transitions={"done": "POUR"})
            self.add("POUR", CBState(_pour), transitions={"done": "ALIGN_PLACE"})
            self.add("ALIGN_PLACE", CBState(_align_place), transitions={"done": "PLACE"})
            self.add("PLACE", CBState(_place), transitions={"done": "succeeded"})


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    PickPourPlaceCereal(robot_instance, "dinner_table").execute()
