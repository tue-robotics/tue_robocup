#
# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import copy
import math
import os
import sys

import rospy
import visualization_msgs.msg
from geometry_msgs.msg import PoseStamped, Vector3

from challenge_clean_the_table.knowledge import (
    ITEM_VECTOR_DICT,
    ITEM_COLOR_DICT,
    ITEMS,
    PLACE_ID,
    PLACE_AREA_ID,
    JOINTS_PRE_PRE_PLACE,
    ITEMS_PLATE,
    JOINTS_PLACE_MUG_BOWL,
    JOINTS_PLACE_PLATE, ITEMS_MUG_BOWL, JOINTS_PLACE_CUTLERY, JOINTS_PRE_PLACE, JOINTS_RETRACT, )
from challenge_clean_the_table.util import item_vector_to_item_frame, item_frame_to_pose
from robot_skills import get_robot
from robot_smach_states.navigation import NavigateToSymbolic, ForceDrive
from robot_smach_states.navigation.control_to_pose import ControlParameters, ControlToPose
from robot_smach_states.util.designators import EdEntityDesignator
from smach import StateMachine, cb_interface, CBState


class PlaceItemInDishwasherRack(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], input_keys=["item_picked"])
        # noinspection PyProtectedMember
        arm = robot.get_arm()._arm

        def send_joint_goal(position_array, wait_for_motion_done=True):
            # noinspection PyProtectedMember
            arm._send_joint_trajectory([position_array], timeout=rospy.Duration(0))
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string, wait_for_motion_done=True):
            arm.gripper.send_goal(open_close_string)

        @cb_interface(outcomes=["done"], input_keys=["item_picked"])
        def _pre_place(user_data):
            item_name = user_data["item_picked"]
            rospy.loginfo(f"Preplacing {item_name}...")

            robot.speech.speak(f"I am going to place the {item_name}", block=False)

            robot.head.look_up()
            robot.head.wait_for_motion_done()

            send_joint_goal(JOINTS_PRE_PRE_PLACE)

            return "done"

        @cb_interface(outcomes=["done"], input_keys=["item_picked"])
        def _align(user_data):
            item_placement_vector = ITEM_VECTOR_DICT[user_data["item_picked"]]
            item_frame = item_vector_to_item_frame(item_placement_vector)

            goal_pose = item_frame_to_pose(item_frame, PLACE_ID)
            rospy.loginfo("Placing {} at {}".format(user_data["item_picked"], goal_pose))
            robot.head.look_down()
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute({})
            return "done"

        @cb_interface(outcomes=["done"], input_keys=["item_picked"])
        def _place_and_retract(user_data):
            rospy.loginfo("Placing...")
            item_name = user_data["item_picked"]

            send_joint_goal(JOINTS_PRE_PLACE)

            if item_name in ITEMS_PLATE:
                send_joint_goal(JOINTS_PLACE_PLATE)
            elif item_name in ITEMS_MUG_BOWL:
                send_joint_goal(JOINTS_PLACE_MUG_BOWL)
            else:
                send_joint_goal(JOINTS_PLACE_CUTLERY)

            rospy.loginfo("Dropping...")
            send_gripper_goal("open")
            robot.head.look_up()
            robot.head.wait_for_motion_done()

            rospy.loginfo("Retract...")
            send_joint_goal(JOINTS_RETRACT, wait_for_motion_done=False)
            robot.base.force_drive(-0.1, 0, 0, 3)  # Drive backwards at 0.1m/s for 3s, so 30cm
            send_gripper_goal("close", wait_for_motion_done=False)
            arm.send_joint_goal("carrying_pose")

            robot.head.reset()

            return "done"

        with self:
            self.add_auto("PRE_PLACE", CBState(_pre_place), ["done"])
            self.add_auto("ALIGN", CBState(_align), ["done"])
            self.add("PLACE_AND_RETRACT", CBState(_place_and_retract), transitions={"done": "succeeded"})


class NavigateToAndPlaceItemInDishwasherRack(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], input_keys=["item_picked"])

        item_frames = {k: item_vector_to_item_frame(v) for k, v in ITEM_VECTOR_DICT.items()}
        item_poses = {k: item_frame_to_pose(v, PLACE_ID) for k, v in item_frames.items()}

        marker_array_pub = rospy.Publisher("/markers", visualization_msgs.msg.MarkerArray, queue_size=1, latch=True)

        _publish_item_poses(marker_array_pub, item_poses)

        place = EdEntityDesignator(robot=robot, uuid=PLACE_ID)

        with self:
            StateMachine.add(
                "NAVIGATE",
                NavigateToSymbolic(robot, {place: PLACE_AREA_ID}, place),
                transitions={
                    "arrived": "PLACE_ITEM_IN_DISHWASHER_RACK",
                    "unreachable": "NAVIGATE_FAILED",
                    "goal_not_defined": "failed",
                },
            )

            StateMachine.add(
                "NAVIGATE_FAILED", ForceDrive(robot, 0.0, 0, 0.5, math.pi / 0.5), transitions={"done": "NAVIGATE"}
            )

            StateMachine.add(
                "PLACE_ITEM_IN_DISHWASHER_RACK",
                PlaceItemInDishwasherRack(robot),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )


def _publish_item_poses(marker_array_pub, items):
    """
    Publishes item poses as a visualization marker array

    :param items: (dict) ...
    """
    array_msg = visualization_msgs.msg.MarkerArray()

    marker_id = 1234
    for k, posestamped in items.items():
        posestamped = posestamped  # type: PoseStamped

        marker_id += 1
        marker_msg = visualization_msgs.msg.Marker()
        marker_msg.header.frame_id = posestamped.header.frame_id
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.id = marker_id
        marker_msg.type = visualization_msgs.msg.Marker.SPHERE
        marker_msg.action = 0
        marker_msg.pose = posestamped.pose
        marker_msg.pose.position.z += 1.0
        marker_msg.scale = Vector3(0.05, 0.05, 0.05)
        marker_msg.color = ITEM_COLOR_DICT[k]
        array_msg.markers.append(marker_msg)

        marker_id += 1
        marker_msg2 = copy.deepcopy(marker_msg)
        marker_msg2.id = marker_id
        marker_msg2.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING
        marker_msg2.pose.position.z += 0.1
        marker_msg2.text = k
        array_msg.markers.append(marker_msg2)

    marker_array_pub.publish(array_msg)


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ITEMS:
        print(f"Please specify an item, e.g {ITEMS}")
        sys.exit(1)

    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])

    robot_instance = get_robot("hero")
    state_machine = NavigateToAndPlaceItemInDishwasherRack(robot_instance)
    state_machine.userdata["item_picked"] = sys.argv[1]
    state_machine.execute()
