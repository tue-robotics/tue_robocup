#
# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn
import os

import rospkg
import rospy

from challenge_serve_breakfast.tuning import REQUIRED_ITEMS, JOINTS_HANDOVER, PICK_ROTATION
from robot_skills import get_robot
from robot_smach_states.navigation import NavigateToSymbolic
from robot_smach_states.util.designators import EdEntityDesignator
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

        def send_joint_goal(position_array, wait_for_motion_done=True):
            # noinspection PyProtectedMember
            arm._send_joint_trajectory([position_array], timeout=rospy.Duration(0))
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
            arm.gripper.send_goal("close", timeout=0.)
            robot.head.look_up()
            vyaw = 0.5
            robot.base.force_drive(0, 0, vyaw, PICK_ROTATION / vyaw)
            return "done"

        @cb_interface(outcomes=["succeeded", "failed"], output_keys=["item_picked"])
        def _ask_user(user_data):
            leftover_items = [item for item in REQUIRED_ITEMS if item not in picked_items]
            if not leftover_items:
                robot.speech.speak("We picked 'm all apparently")
                return "failed"

            item_name = leftover_items[0]

            send_joint_goal(JOINTS_HANDOVER, wait_for_motion_done=False)

            picked_items.append(item_name)

            robot.speech.speak("Please put the {} in my gripper, like this".format(item_name), block=False)
            show_image("challenge_serve_breakfast", item_img_dict[item_name])

            send_gripper_goal("open")
            rospy.sleep(5.0)
            robot.speech.speak("Thanks for that!", block=False)
            send_gripper_goal("close", max_torque=0.6)
            robot.head.reset()

            # Set output data
            user_data["item_picked"] = item_name

            arm.send_joint_goal("carrying_pose", timeout=0.)

            return "succeeded"

        with self:
            self.add("ROTATE", CBState(_rotate), transitions={"done": "ASK_USER"})
            self.add("ASK_USER", CBState(_ask_user), transitions={"succeeded": "succeeded", "failed": "failed"})


class NavigateToAndPickItem(StateMachine):
    def __init__(self, robot, pick_spot_id, pick_spot_navigation_area):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], output_keys=["item_picked"])

        pick_spot = EdEntityDesignator(robot=robot, uuid=pick_spot_id)

        with self:
            StateMachine.add(
                "NAVIGATE_TO_PICK_SPOT",
                NavigateToSymbolic(robot, {pick_spot: pick_spot_navigation_area}, pick_spot),
                transitions={"arrived": "PICK_ITEM", "unreachable": "failed", "goal_not_defined": "failed"},
            )

            StateMachine.add("PICK_ITEM", PickItem(robot), transitions={"succeeded": "succeeded", "failed": "failed"})


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    robot_instance.reset()
    NavigateToAndPickItem(robot_instance, "dinner_table", "in_front_of").execute()
