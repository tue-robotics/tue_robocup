#
# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import copy
import os
import sys

import rospy
import visualization_msgs.msg
from geometry_msgs.msg import PoseStamped, Vector3

from challenge_serve_breakfast.tuning import (
    JOINTS_PRE_PRE_PLACE,
    JOINTS_PRE_PLACE_HORIZONTAL,
    JOINTS_PRE_PLACE_VERTICAL,
    ITEM_VECTOR_DICT,
    item_vector_to_item_frame,
    item_frame_to_pose,
    JOINTS_PLACE_HORIZONTAL,
    JOINTS_PLACE_VERTICAL,
    JOINTS_RETRACT,
    COLOR_DICT, REQUIRED_ITEMS,
)
from robot_skills import get_robot
from robot_smach_states.human_interaction import Say
from robot_smach_states.navigation import NavigateToSymbolic, ForceDrive
from robot_smach_states.navigation.control_to_pose import ControlParameters, ControlToPose
from robot_smach_states.util.designators import EdEntityDesignator
from robot_smach_states.utility import WaitTime
from smach import StateMachine, cb_interface, CBState


class PlaceItemOnTable(StateMachine):
    def __init__(self, robot, table_id):
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
            if wait_for_motion_done:
                rospy.sleep(1.0)  # Does not work with motion_done apparently

        @cb_interface(outcomes=["done"], input_keys=["item_picked"])
        def _pre_place(user_data):
            item_name = user_data["item_picked"]
            rospy.loginfo(f"Preplacing {item_name}...")

            robot.speech.speak(f"I am going to place the {item_name}", block=False)

            robot.head.look_up()
            robot.head.wait_for_motion_done()

            send_joint_goal(JOINTS_PRE_PRE_PLACE)

            if item_name in ["milk_carton", "cereal_box"]:
                send_joint_goal(JOINTS_PRE_PLACE_HORIZONTAL)
            else:
                send_joint_goal(JOINTS_PRE_PLACE_VERTICAL)

            return "done"

        @cb_interface(outcomes=["done"], input_keys=["item_picked"])
        def _align_with_table(user_data):
            item_placement_vector = ITEM_VECTOR_DICT[user_data["item_picked"]]
            item_frame = item_vector_to_item_frame(item_placement_vector)

            goal_pose = item_frame_to_pose(item_frame, table_id)
            rospy.loginfo("Placing {} at {}".format(user_data["item_picked"], goal_pose))
            robot.head.look_down()
            robot.head.wait_for_motion_done()
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute({})
            return "done"

        @cb_interface(outcomes=["done"], input_keys=["item_picked"])
        def _place_and_retract(user_data):
            rospy.loginfo("Placing...")
            item_name = user_data["item_picked"]
            if item_name in ["milk_carton", "cereal_box"]:
                send_joint_goal(JOINTS_PLACE_HORIZONTAL)
            else:
                send_joint_goal(JOINTS_PLACE_VERTICAL)

            rospy.loginfo("Dropping...")
            send_gripper_goal("open")
            robot.head.look_up()
            robot.head.wait_for_motion_done()

            if item_name != "cereal_box":
                rospy.loginfo("Retract...")
                send_joint_goal(JOINTS_RETRACT, wait_for_motion_done=False)
                robot.base.force_drive(-0.1, 0, 0, 3)  # Drive backwards at 0.1m/s for 3s, so 30cm
                send_gripper_goal("close", wait_for_motion_done=False)
                arm.send_joint_goal("carrying_pose")

            robot.head.reset()

            return "done"

        with self:
            self.add_auto("PRE_PLACE", CBState(_pre_place), ["done"])
            self.add_auto("ALIGN_WITH_TABLE", CBState(_align_with_table), ["done"])
            self.add("PLACE_AND_RETRACT", CBState(_place_and_retract), transitions={"done": "succeeded"})


class NavigateToAndPlaceItemOnTable(StateMachine):
    def __init__(self, robot, table_id, table_close_navigation_area):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], input_keys=["item_picked"])

        item_frames = {k: item_vector_to_item_frame(v) for k, v in ITEM_VECTOR_DICT.items()}
        item_poses = {k: item_frame_to_pose(v, table_id) for k, v in item_frames.items()}

        marker_array_pub = rospy.Publisher('/markers', visualization_msgs.msg.MarkerArray, queue_size=1, latch=True)

        _publish_item_poses(marker_array_pub, item_poses)

        table = EdEntityDesignator(robot=robot, uuid=table_id)

        with self:
            StateMachine.add(
                "NAVIGATE_TO_TABLE_CLOSE",
                NavigateToSymbolic(robot, {table: table_close_navigation_area}, table),
                transitions={
                    "arrived": "PLACE_ITEM_ON_TABLE",
                    "unreachable": "SAY_PICK_AWAY_THE_CHAIR",
                    "goal_not_defined": "failed",
                },
            )

            StateMachine.add(
                "FORCE_DRIVE", ForceDrive(robot, -0.1, 0, 0, 5.0), transitions={"done": "SAY_PICK_AWAY_THE_CHAIR"}
            )

            StateMachine.add(
                "SAY_PICK_AWAY_THE_CHAIR",
                Say(robot, "Please pick away the chair, I cannot get close enough to the {}".format(table_id)),
                transitions={"spoken": "WAIT_FOR_PICK_AWAY_CHAIR"},
            )

            StateMachine.add(
                "WAIT_FOR_PICK_AWAY_CHAIR",
                WaitTime(robot, 5),
                transitions={"waited": "SAY_THANKS", "preempted": "failed"},
            )

            StateMachine.add(
                "SAY_THANKS", Say(robot, "Thank you darling"), transitions={"spoken": "NAVIGATE_TO_TABLE_CLOSE"}
            )

            StateMachine.add(
                "PLACE_ITEM_ON_TABLE",
                PlaceItemOnTable(robot, table_id),
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
        marker_msg.color = COLOR_DICT[k]
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
    if len(sys.argv) < 2 or sys.argv[1] not in REQUIRED_ITEMS:
        print(f"Please specify an item, e.g {REQUIRED_ITEMS}")
        sys.exit(1)

    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])

    robot_instance = get_robot("hero")
    state_machine = NavigateToAndPlaceItemOnTable(robot_instance, "dinner_table", "in_front_of")
    state_machine.userdata["item_picked"] = sys.argv[1]
    state_machine.execute()
