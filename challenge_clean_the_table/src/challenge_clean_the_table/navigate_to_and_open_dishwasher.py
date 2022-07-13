#
# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn
import math
import os

import rospy
import visualization_msgs.msg
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA

from challenge_clean_the_table.knowledge import (
    DISHWASHER_ID,
    DISHWASHER_AREA_ID,
    OPEN_DISHWASHER_VECTOR,
    OPEN_DISHWASHER_VECTOR_OPEN,
    JOINTS_OPEN_DISHWASHER,
)
from challenge_clean_the_table.util import item_vector_to_item_frame, item_frame_to_pose
from robot_skills import get_robot
from robot_smach_states.navigation import NavigateToSymbolic, ForceDrive
from robot_smach_states.navigation.control_to_pose import ControlToPose, ControlParameters
from robot_smach_states.util.designators import EdEntityDesignator
from smach import StateMachine, cb_interface, CBState


class OpenDishwasher(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["succeeded"])
        # noinspection PyProtectedMember
        arm = robot.get_arm()._arm

        def send_joint_goal(position_array, wait_for_motion_done=True):
            # noinspection PyProtectedMember
            arm._send_joint_trajectory([position_array], timeout=rospy.Duration(0))
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string, max_torque=0.6):
            arm.gripper.send_goal(open_close_string, max_torque=max_torque)

        @cb_interface(outcomes=["done"])
        def _align_pre(_):
            item_frame = item_vector_to_item_frame(OPEN_DISHWASHER_VECTOR_OPEN)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            robot.head.look_down()
            robot.speech.speak("Looking for the handle", block=False)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute({})
            return "done"

        @cb_interface(outcomes=["done"])
        def _pre_grasp(_):
            send_gripper_goal("open")
            send_joint_goal(JOINTS_OPEN_DISHWASHER)
            return "done"

        @cb_interface(outcomes=["done"])
        def _align_handle(_):
            item_frame = item_vector_to_item_frame(OPEN_DISHWASHER_VECTOR)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            robot.head.look_down()
            robot.speech.speak("Grabbing the handle", block=False)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute({})
            return "done"

        @cb_interface(outcomes=["done"])
        def _grab_handle(_):
            send_gripper_goal("close", max_torque=0.2)
            rospy.sleep(1.0)
            return "done"

        @cb_interface(outcomes=["done"])
        def _open(_):
            item_frame = item_vector_to_item_frame(OPEN_DISHWASHER_VECTOR_OPEN)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute({})
            arm.send_joint_goal("carrying_pose", timeout=0.0)
            robot.speech.speak("Hopefully the dishwasher is now open. Thanks guys.", block=False)
            return "done"

        with self:
            self.add("ALIGN_PRE", CBState(_align_pre), transitions={"done": "PRE_GRASP"})
            self.add("PRE_GRASP", CBState(_pre_grasp), transitions={"done": "ALIGN_HANDLE"})
            self.add("ALIGN_HANDLE", CBState(_align_handle), transitions={"done": "GRAB_HANDLE"})
            self.add("GRAB_HANDLE", CBState(_grab_handle), transitions={"done": "OPEN"})
            self.add("OPEN", CBState(_open), transitions={"done": "succeeded"})


class NavigateToAndOpenDishwasher(StateMachine):
    marker_array_pub = rospy.Publisher("/markers", visualization_msgs.msg.MarkerArray, queue_size=1, latch=True)

    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        dishwasher = EdEntityDesignator(robot=robot, uuid=DISHWASHER_ID)

        array_msg = visualization_msgs.msg.MarkerArray()
        marker_msg = visualization_msgs.msg.Marker()
        marker_msg.header.frame_id = DISHWASHER_ID
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.ns = "open_dishwasher_vector"
        marker_msg.type = visualization_msgs.msg.Marker.SPHERE
        marker_msg.action = 0
        marker_msg.pose = item_frame_to_pose(item_vector_to_item_frame(OPEN_DISHWASHER_VECTOR), DISHWASHER_ID).pose
        marker_msg.pose.position.z += 1.0
        marker_msg.scale = Vector3(0.05, 0.05, 0.05)
        marker_msg.color = ColorRGBA(0.2, 1.0, 0.2, 1)
        array_msg.markers.append(marker_msg)

        NavigateToAndOpenDishwasher.marker_array_pub.publish(array_msg)

        with self:
            StateMachine.add(
                "NAVIGATE_TO_DISHWASHER",
                NavigateToSymbolic(robot, {dishwasher: DISHWASHER_AREA_ID}, dishwasher),
                transitions={
                    "arrived": "OPEN_DISHWASHER",
                    "unreachable": "NAVIGATE_TO_DISHWASHER_FAILED",
                    "goal_not_defined": "failed",
                },
            )

            StateMachine.add(
                "NAVIGATE_TO_DISHWASHER_FAILED",
                ForceDrive(robot, 0.0, 0, 0.5, math.pi / 0.5),
                transitions={"done": "NAVIGATE_TO_DISHWASHER"},
            )

            StateMachine.add("OPEN_DISHWASHER", OpenDishwasher(robot), transitions={"succeeded": "succeeded"})


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    robot_instance.reset()
    NavigateToAndOpenDishwasher(robot_instance).execute()
