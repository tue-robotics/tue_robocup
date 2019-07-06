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


class GrabRack(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=['done'])
        arm = robot.get_arm()._arm

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

        @cb_interface(outcomes=['done'])
        def _ask_user(user_data):
            return 'done'

        with self:
            self.add('ASK_USER', CBState(_ask_user), transitions={'succeeded': 'succeeded', 'failed': 'failed'})


class NavigateToAndGrabRack(StateMachine):
    def __init__(self, robot, rack_id, rack_navigation_area):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        rack = EdEntityDesignator(robot=robot, id=rack_id)

        with self:
            StateMachine.add("NAVIGATE_TO_RACK",
                             NavigateToSymbolic(robot, {rack: rack_navigation_area}, rack),
                             transitions={'arrived': 'GRAB_RACK',
                                          'unreachable': 'failed',
                                          'goal_not_defined': 'failed'})

            StateMachine.add("GRAB_RACK", GrabRack(robot),
                             transitions={'done': 'succeeded'})


if __name__ == '__main__':
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    hero = Hero()
    hero.reset()
    GrabRack(hero).execute()
