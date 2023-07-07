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
    OPEN_DISHWASHER_VECTOR_OPEN1,
    OPEN_DISHWASHER_VECTOR_OPEN2,
    OPEN_DISHWASHER_VECTOR_OPEN3,
    OPEN_DISHWASHER_VECTOR_OPEN4,
    OPEN_DISHWASHER_VECTOR_OPEN5,
    OPEN_DISHWASHER_VECTOR_OPEN6,
    PULL_DISHWASHER_RACK_PREPARE,
    PULL_DISHWASHER_RACK_PREPARE_START,
    PULL_DISHWASHER_RACK_OUT1,
    PULL_DISHWASHER_RACK_OUT2,
    JOINTS_OPEN_DISHWASHER_PRE,
    JOINTS_OPEN_DISHWASHER,
    JOINTS_OPEN_DISHWASHER1,
    JOINTS_OPEN_DISHWASHER2,
    JOINTS_OPEN_DISHWASHER3,
    JOINTS_OPEN_DISHWASHER4,
    JOINTS_OPEN_DISHWASHER5,
    JOINTS_OPEN_DISHWASHER6,
    JOINTS_PULL_RACK_PREPARE,
    JOINTS_PULL_RACK_DOWN,
)
from challenge_clean_the_table.util import item_vector_to_item_frame, item_vector_to_item_frame_2d, item_frame_to_pose
from robot_skills import get_robot
from robot_skills.simulation.sim_mode import is_sim_mode
from robot_smach_states.human_interaction import AskYesNo, AskYesNoPicoVoice, AskContinue, Say
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
            arm._send_joint_trajectory([position_array], timeout=0)
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string, max_torque=0.6):
            arm.gripper.send_goal(open_close_string, max_torque=max_torque)

        @cb_interface(outcomes=["done"])
        def _align_pre(_):
            item_frame = item_vector_to_item_frame(OPEN_DISHWASHER_VECTOR_OPEN3)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            robot.head.look_down()
            robot.speech.speak("Looking for the handle", block=False)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute()
            return "done"

        @cb_interface(outcomes=["done"])
        def _pre_grasp(_):
            send_gripper_goal("open")
            send_joint_goal(JOINTS_OPEN_DISHWASHER_PRE)
            return "done"

        @cb_interface(outcomes=["done"])
        def _align_handle(_):
            item_frame = item_vector_to_item_frame(OPEN_DISHWASHER_VECTOR)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            robot.head.look_down()
            robot.speech.speak("Grabbing the handle", block=False)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute()
            send_joint_goal(JOINTS_OPEN_DISHWASHER)
            return "done"

        @cb_interface(outcomes=["done"])
        def _grab_handle(_):
            send_gripper_goal("close", max_torque=0.6)
            rospy.sleep(1.0)
            send_gripper_goal("close", max_torque=0.6)
            rospy.sleep(1.0)
            return "done"

        @cb_interface(outcomes=["done"])
        def _open(_):
            robot.speech.speak("Pulling the door open", block=False)
            item_frame = item_vector_to_item_frame_2d(OPEN_DISHWASHER_VECTOR_OPEN1)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute()
            send_joint_goal(JOINTS_OPEN_DISHWASHER1)
            item_frame = item_vector_to_item_frame_2d(OPEN_DISHWASHER_VECTOR_OPEN2)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute()
            send_joint_goal(JOINTS_OPEN_DISHWASHER2)
            item_frame = item_vector_to_item_frame_2d(OPEN_DISHWASHER_VECTOR_OPEN3)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute()
            send_joint_goal(JOINTS_OPEN_DISHWASHER3)
            send_gripper_goal("open")

            # Drive away from the dishwasher
            item_frame = item_vector_to_item_frame_2d(OPEN_DISHWASHER_VECTOR_OPEN4)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute()

            # Straight arm
            send_joint_goal(JOINTS_OPEN_DISHWASHER4)

            # Move above door
            item_frame = item_vector_to_item_frame_2d(OPEN_DISHWASHER_VECTOR_OPEN5)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute()

            # Lower arm
            robot.speech.speak("Pushing the door down", block=False)
            send_joint_goal(JOINTS_OPEN_DISHWASHER5)

            # Drive a bit away
            item_frame = item_vector_to_item_frame_2d(OPEN_DISHWASHER_VECTOR_OPEN6)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute()

            # Lower arm fully
            send_joint_goal(JOINTS_OPEN_DISHWASHER6)
            rospy.sleep(0.5)

            # Increase height of arm again
            send_joint_goal(JOINTS_PULL_RACK_PREPARE)

            robot.speech.speak("Hopefully the dishwasher is now open.", block=False)
            return "done"

        @cb_interface(outcomes=["done"])
        def _pull_rack_prepare(_):
            # Prepare above door
            send_joint_goal(JOINTS_PULL_RACK_PREPARE)

            # item_frame = item_vector_to_item_frame_2d(OPEN_DISHWASHER_VECTOR_OPEN6)
            # goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            # ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute()

            # Move to the side
            item_frame = item_vector_to_item_frame_2d(PULL_DISHWASHER_RACK_PREPARE)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute()

            # Move closer at the side of the dishwasher
            item_frame = item_vector_to_item_frame_2d(PULL_DISHWASHER_RACK_PREPARE_START)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute()

            return "done"

        @cb_interface(outcomes=["done"])
        def _pull_rack(_):
            robot.speech.speak("Pulling the rack", block=False)
            # Move arm in
            item_frame = item_vector_to_item_frame_2d(PULL_DISHWASHER_RACK_OUT1)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute()

            send_gripper_goal("close", max_torque=0.2)

            # Move down
            send_joint_goal(JOINTS_PULL_RACK_DOWN)
            rospy.sleep(0.5)

            # Pull out
            item_frame = item_vector_to_item_frame_2d(PULL_DISHWASHER_RACK_OUT2)
            goal_pose = item_frame_to_pose(item_frame, DISHWASHER_ID)
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.25, 0.25, 0.3, 0.02, 0.1)).execute()

            send_gripper_goal("open")

            return "done"

        with self:
            self.add("ALIGN_PRE", CBState(_align_pre), transitions={"done": "PRE_GRASP"})
            self.add("PRE_GRASP", CBState(_pre_grasp), transitions={"done": "ALIGN_HANDLE"})
            self.add("ALIGN_HANDLE", CBState(_align_handle), transitions={"done": "GRAB_HANDLE"})
            self.add("GRAB_HANDLE", CBState(_grab_handle), transitions={"done": "OPEN"})
            self.add("OPEN", CBState(_open), transitions={"done": "PULL_RACK_PREPARE"})
            self.add("SAY_DOOR_CORRECT",
                     Say(robot, "Did I open the door fully, Yes or No?"),
                     transitions={"spoken": "YES_OR_NO_DOOR"})
            if is_sim_mode():
                self.add("YES_OR_NO_DOOR", AskYesNo(robot),
                         transitions={"yes": "PULL_RACK_PREPARE",
                                      "no": "ASK_TO_OPEN_DOOR_CORRECTLY",
                                      "no_result": "ASK_TO_OPEN_DOOR_CORRECTLY"})
            else:
                self.add("YES_OR_NO_DOOR", AskYesNoPicoVoice(robot),
                         transitions={"yes": "PULL_RACK_PREPARE",
                                      "no": "ASK_TO_OPEN_DOOR_CORRECTLY",
                                      "no_result": "ASK_TO_OPEN_DOOR_CORRECTLY"})
            self.add("ASK_TO_OPEN_DOOR_CORRECTLY",
                     Say(robot, "Please open the door fully and let me know when I should continue"),
                     transitions={"spoken": "DOOR_CONTINUE"})

            self.add("DOOR_CONTINUE", AskContinue(robot, timeout=20),
                     transitions={"continue": "PULL_RACK", "no_response": "PULL_RACK"})

            self.add("PULL_RACK_PREPARE", CBState(_pull_rack_prepare), transitions={"done": "PULL_RACK"})
            self.add("PULL_RACK", CBState(_pull_rack), transitions={"done": "succeeded"})

            self.add("SAY_RACK_CORRECT", Say(robot, "Did I pull the rack fully, Yes or No?"),
                     transitions={"spoken": "YES_OR_NO_RACK"})
            if is_sim_mode():
                self.add("YES_OR_NO_RACK", AskYesNo(robot),
                         transitions={"yes": "succeeded",
                                      "no": "ASK_TO_OPEN_DOOR_CORRECTLY",
                                      "no_result": "ASK_TO_OPEN_DOOR_CORRECTLY"})
            else:
                self.add("YES_OR_NO_RACK", AskYesNoPicoVoice(robot),
                         transitions={"yes": "PULL_RACK_PREPARE",
                                      "no": "ASK_TO_PULL_RACK_CORRECTLY",
                                      "no_result": "ASK_TO_PULL_RACK_CORRECTLY"})
            self.add("ASK_TO_PULL_RACK_CORRECTLY",
                     Say(robot, "Please pull out the rack fully and let me know when I should continue"),
                     transitions={"spoken": "RACK_CONTINUE"})

            self.add("RACK_CONTINUE", AskContinue(robot, timeout=20),
                     transitions={"continue": "succeeded", "no_response": "succeeded"})


class NavigateToAndOpenDishwasher(StateMachine):
    marker_array_pub = rospy.Publisher("/markers", visualization_msgs.msg.MarkerArray, queue_size=1, latch=True)

    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        dishwasher = EdEntityDesignator(robot=robot, uuid=DISHWASHER_ID)

        @cb_interface(outcomes=["done"])
        def _publish_item_poses(userdata):
            """
            Publishes item poses as a visualization marker array
            """
            array_msg = visualization_msgs.msg.MarkerArray()
            marker_msg = visualization_msgs.msg.Marker()
            marker_msg.header.frame_id = DISHWASHER_ID
            marker_msg.header.stamp = rospy.Time()
            marker_msg.ns = "open_dishwasher_vector"
            marker_msg.type = visualization_msgs.msg.Marker.SPHERE
            marker_msg.action = 0
            marker_msg.pose = item_frame_to_pose(item_vector_to_item_frame(OPEN_DISHWASHER_VECTOR), DISHWASHER_ID).pose
            marker_msg.pose.position.z = 0.5
            marker_msg.scale = Vector3(0.05, 0.05, 0.05)
            marker_msg.color = ColorRGBA(0.2, 1.0, 0.2, 1)
            array_msg.markers.append(marker_msg)

            NavigateToAndOpenDishwasher.marker_array_pub.publish(array_msg)
            return "done"

        with self:
            StateMachine.add_auto("PUBLISH_MARKERS", CBState(_publish_item_poses), ["done"])
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
    sm = NavigateToAndOpenDishwasher(robot_instance)
    sm.execute()
