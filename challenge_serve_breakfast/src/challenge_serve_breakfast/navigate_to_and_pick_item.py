#
# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn
import os

import rospkg
import rospy

from challenge_serve_breakfast.tuning import REQUIRED_ITEMS, JOINTS_HANDOVER
from robot_skills import get_robot
from robot_skills.arm.arms import GripperTypes

from pykdl_ros import VectorStamped
from robot_smach_states.human_interaction import Say
from robot_smach_states.manipulation.active_grasp_detector import ActiveGraspDetector

from robot_smach_states.navigation import NavigateToSymbolic
from robot_smach_states.util.designators import EdEntityDesignator, ArmDesignator
from smach import StateMachine, cb_interface, CBState

item_img_dict = {
    "bowl": "images/bowl.jpg",
    "spoon": "images/spoon.jpg",
    "cereal_box": "images/cereal_box.jpg",
    "milk_carton": "images/milk_carton.jpg",
}


class PickItem(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], output_keys=["item_picked"])
        # noinspection PyProtectedMember
        arm = robot.get_arm()._arm
        picked_items = []
        armdes = ArmDesignator(robot, {"required_gripper_types": [GripperTypes.GRASPING]})

        def send_joint_goal(position_array, wait_for_motion_done=True):
            # noinspection PyProtectedMember
            arm._send_joint_trajectory([position_array], timeout=0.0)
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string, max_torque=0.1):
            arm.gripper.send_goal(open_close_string, max_torque=max_torque)

        def show_image(package_name, path_to_image_in_package):
            path = os.path.join(rospkg.RosPack().get_path(package_name), path_to_image_in_package)
            if not os.path.exists(path):
                rospy.logerr("Image path {} does not exist".format(path))
            else:
                try:
                    rospy.loginfo("Showing {}".format(path))
                    robot.hmi.show_image(path, 10)
                except Exception as e:
                    rospy.logerr("Could not show image {}: {}".format(path, e))
            return "succeeded"

        @cb_interface(outcomes=["done"])
        def _rotate(_):
            arm.gripper.send_goal("close", timeout=0.0)

            # Look to the operator
            robot.head.look_at_point(VectorStamped.from_xyz(0.2, -0.2, 1.75, stamp=rospy.Time.now(),
                                                            frame_id=robot.base_link_frame))
            robot.head.wait_for_motion_done()
            return "done"

        @cb_interface(outcomes=["done"])
        def _handover_pose(user_data):
            send_joint_goal(JOINTS_HANDOVER, wait_for_motion_done=False)
            return "done"

        @cb_interface(outcomes=["succeeded", "failed"], output_keys=["item_picked"])
        def _ask_user(user_data):
            leftover_items = [item for item in REQUIRED_ITEMS if item not in picked_items]
            if not leftover_items:
                robot.speech.speak("We picked 'm all apparently")
                return "failed"

            item_name = leftover_items[0]

            robot.speech.speak("Please put the {} in my gripper, like this".format(item_name), block=False)
            show_image("challenge_serve_breakfast", item_img_dict[item_name])

            send_gripper_goal("open")
            rospy.sleep(7.0)
            send_gripper_goal("close", max_torque=0.6)
            robot.head.reset()

            # Set output data
            user_data["item_picked"] = item_name
            return "succeeded"

        @cb_interface(outcomes=["done"])
        def _carrying_pose(user_data):
            arm.send_joint_goal("carrying_pose", timeout=0.)
            robot.speech.speak("Thanks for that!", block=False)
            return "done"

        @cb_interface(outcomes=["done"], input_keys=["item_picked"])
        def _remember_item(user_data):
            picked_items.append(user_data["item_picked"])
            return "done"

        with self:
            self.add("ROTATE", CBState(_rotate), transitions={"done": "HANDOVER_POSE"})
            self.add("HANDOVER_POSE", CBState(_handover_pose), transitions={"done": "ASK_USER"})
            self.add("ASK_USER", CBState(_ask_user),
                     transitions={"succeeded": "CHECK_PICK_SUCCESSFUL", "failed": "failed"})
            self.add("CHECK_PICK_SUCCESSFUL",
                     ActiveGraspDetector(robot, armdes),
                     transitions={'true': "ADD_MM_TO_LIST",
                                  'false': "SAY_SOMETHING_WENT_WRONG",
                                  'failed': "failed",
                                  'cannot_determine': "SAY_SOMETHING_WENT_WRONG"}
                     )
            self.add("SAY_SOMETHING_WENT_WRONG", Say(robot, "Oops, it seems I missed it. Lets try again"),
                     transitions={"spoken": "ASK_USER2"})
            self.add("ASK_USER2", CBState(_ask_user), transitions={"succeeded": "ADD_ITEM_TO_LIST", "failed": "failed"})
            self.add("ADD_ITEM_TO_LIST", CBState(_remember_item), transitions={"done": "CARRYING_POSE"})
            self.add("CARRYING_POSE", CBState(_carrying_pose), transitions={"done": "succeeded"})


class NavigateToAndPickItem(StateMachine):
    def __init__(self, robot, pick_spot_id, pick_spot_navigation_area):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], output_keys=["item_picked"])

        pick_spot = EdEntityDesignator(robot=robot, uuid=pick_spot_id)

        with self:
            StateMachine.add(
                "NAVIGATE_TO_PICK_SPOT",
                NavigateToSymbolic(robot, {pick_spot: pick_spot_navigation_area}, pick_spot, speak=False),
                transitions={"arrived": "PICK_ITEM", "unreachable": "failed", "goal_not_defined": "failed"},
            )

            StateMachine.add("PICK_ITEM", PickItem(robot), transitions={"succeeded": "succeeded", "failed": "failed"})


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    robot_instance.reset()
    NavigateToAndPickItem(robot_instance, "dinner_table", "in_front_of").execute()
