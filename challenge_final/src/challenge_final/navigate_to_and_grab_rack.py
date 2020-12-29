#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn
import math
import os

import rospkg
import rospy
from dynamic_reconfigure.client import Client
from geometry_msgs.msg import PoseStamped, Quaternion
from robot_skills import Hero
from robot_smach_states import NavigateToSymbolic
from robot_smach_states.navigation.control_to_pose import ControlParameters, ControlToPose
from robot_smach_states.util.designators import EdEntityDesignator
from smach import StateMachine, cb_interface, CBState
from tf.transformations import quaternion_from_euler


class GrabRack(StateMachine):
    def __init__(self, robot, rack_id):
        StateMachine.__init__(self, outcomes=['done'])
        public_arm = robot.get_arm()
        arm = public_arm._arm

        local_client = Client("/hero/local_planner/local_costmap")
        local_client.update_configuration({"footprint": []})
        global_client = Client("/hero/global_planner/global_costmap")
        global_client.update_configuration({"footprint": []})

        def send_joint_goal(position_array, wait_for_motion_done=True):
            arm._send_joint_trajectory([position_array], timeout=rospy.Duration(0))
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string, max_torque=1.0):
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
        def _pre_grab(_):
            robot.speech.speak("Hey, I found myself a nice rack")
            send_joint_goal([0, 0, 0, 0, 0])
            send_joint_goal([0.69, 0, 0, 0, 0])
            rospy.sleep(1.0)
            send_gripper_goal("open")
            send_joint_goal([0.69, -1.77, 0, -1.37, 1.57])

            return 'done'

        @cb_interface(outcomes=['done'])
        def _align(_):
            robot.head.look_down()
            robot.speech.speak("Let's see what I can do with this")
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = rack_id
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi))
            goal_pose.pose.position.x = 0.45
            goal_pose.pose.position.y = 0.09
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.05)).execute({})
            return 'done'

        @cb_interface(outcomes=['done'])
        def _grab(_):
            robot.speech.speak("Grasping the trolley")
            rospy.sleep(4.0)
            send_joint_goal([0.61, -1.77, 0, -1.37, 1.57])
            send_gripper_goal("close")

            base_footprint = [[0.491716563702, 0.284912616014], [0.504091262817, -0.264433205128],
                              [0.00334876775742, -0.259195685387], [-0.17166364193, -0.19022783637],
                              [-0.239429235458, -0.07719720155], [-0.237978458405, 0.0547728165984],
                              [-0.180378556252, 0.164403796196], [-0.0865250825882, 0.221749901772],
                              [0.00969874858856, 0.260340631008]]
            local_client.update_configuration({"footprint": base_footprint})
            global_client.update_configuration({"footprint": base_footprint})
            robot.head.cancel_goal()
            robot.publish_rack()

            return 'done'

        @cb_interface(outcomes=['done'])
        def _retract(_):
            robot.head.look_down()
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = rack_id
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, - math.pi / 2))
            goal_pose.pose.position.x = 0.55
            goal_pose.pose.position.y = -0.1
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute({})
            return 'done'

        with self:
            self.add('PRE_GRAB', CBState(_pre_grab), transitions={'done': 'ALIGN_GRAB'})
            self.add('ALIGN_GRAB', CBState(_align), transitions={'done': 'GRAB'})
            self.add('GRAB', CBState(_grab), transitions={'done': 'RETRACT'})
            self.add('RETRACT', CBState(_retract), transitions={'done': 'done'})


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

            StateMachine.add("GRAB_RACK", GrabRack(robot, rack_id),
                             transitions={'done': 'succeeded'})


if __name__ == '__main__':
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    hero = Hero()
    hero.reset()
    NavigateToAndGrabRack(hero, "rack", "in_front_of").execute()
