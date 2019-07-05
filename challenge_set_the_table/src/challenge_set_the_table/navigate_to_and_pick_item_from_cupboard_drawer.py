#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import os

import rospy
import rospkg
from robot_skills import Hero
from robot_smach_states import NavigateToSymbolic
from robot_smach_states.util.designators import EdEntityDesignator
from smach import StateMachine, cb_interface, CBState

item_img_dict = {
    "plate": 'images/plate.jpg',
    "cup": 'images/cup.jpg',
    "bowl": 'images/bowl.jpg',
    "napkin": 'images/napkin.jpg',
    "knife": 'images/knife.jpg',
    "fork": 'images/fork.jpg',
    "spoon": 'images/spoon.jpg'
}

plate_handover = [0.4, -0.2, 0.0, -1.37, -1.5]

class PickItemFromCupboardDrawer(StateMachine):
    def __init__(self, robot, cupboard_id, required_items):
        StateMachine.__init__(self, outcomes=['succeeded', 'failed'], output_keys=["item_picked"])
        arm = robot.get_arm()._arm
        picked_items = []

        def send_joint_goal(position_array, wait_for_motion_done=True):
            arm._send_joint_trajectory([position_array], timeout=rospy.Duration(0))
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string, max_torque=0.1):
            arm.send_gripper_goal(open_close_string, max_torque=max_torque)
            rospy.sleep(1.0)  # Does not work with motion_done apparently

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
            return 'succeeded'

        @cb_interface(outcomes=['succeeded', 'failed'], output_keys=["item_picked"])
        def _ask_user(user_data):
            leftover_items = [item for item in required_items if item not in picked_items]
            if not leftover_items:
                robot.speech.speak("We picked 'm all apparently")
                return 'failed'


            item_name = leftover_items[0]

            if item_name == 'plate':
                send_joint_goal(plate_handover)
            else:
                arm.send_joint_goal("carrying_pose")
            picked_items.append(item_name)

            robot.speech.speak("Please put the {} in my gripper, like this".format(item_name), block=False)
            show_image('challenge_set_the_table', item_img_dict[item_name])

            send_gripper_goal("open")
            rospy.sleep(5.0)
            robot.speech.speak("Thanks for that!", block=False)
            if item_name == 'plate':
                send_gripper_goal("close", max_torque=0.7)
            else:
                send_gripper_goal("close")

            # Set output data
            user_data['item_picked'] = item_name

            arm.send_joint_goal("carrying_pose")

            return 'succeeded'

        with self:
            self.add('ASK_USER', CBState(_ask_user), transitions={'succeeded': 'succeeded', 'failed': 'failed'})


class NavigateToAndPickItemFromCupboardDrawer(StateMachine):
    def __init__(self, robot, cupboard_id, cupboard_navigation_area, required_items):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], output_keys=["item_picked"])

        cupboard = EdEntityDesignator(robot=robot, id=cupboard_id)

        with self:
            StateMachine.add("NAVIGATE_TO_CUPBOARD",
                             NavigateToSymbolic(robot, {cupboard: cupboard_navigation_area}, cupboard),
                             transitions={'arrived': 'PICK_ITEM_FROM_CUPBOARD',
                                          'unreachable': 'failed',
                                          'goal_not_defined': 'failed'})

            StateMachine.add("PICK_ITEM_FROM_CUPBOARD", PickItemFromCupboardDrawer(robot, cupboard_id, required_items),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


if __name__ == '__main__':
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    hero = Hero()
    hero.reset()
    NavigateToAndPickItemFromCupboardDrawer(hero, 'cupboard', 'aside_of').execute()
