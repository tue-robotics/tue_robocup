#! /usr/bin/env python
import rospy
import smach

import robot_skills.util.transformations as transformations
from robot_smach_states.navigation import NavigateToGrasp

from robot_smach_states.state import State

# ----------------------------------------------------------------------------------------------------

class Put(State):

    def __init__(self, robot, item_to_place, placement_pose, arm):
        State.__init__(self, locals(), outcomes=['succeeded', 'failed'])

    def run(self, robot, item_to_place, placement_pose, arm):
        rospy.loginfo("Placing")

        # placement_pose is a PoseStamped
        placement_in_baselink = transformations.tf_transform(   placement_pose.pose.position,
                                                                placement_pose.header.frame_id,
                                                                robot.robot_name+'/base_link',
                                                                tf_listener=robot.tf_listener)

        # Torso to highest position
        robot.spindle.high()

        if arm.side == "left":
            goal_y = 0.2
        else:
            goal_y = -0.2

        try:
            height = placement_in_baselink.pose.position.z
        except KeyError:
            height = 0.8

        dx = 0.5

        x = 0.2
        while x <= dx:
            if not arm.send_goal(x, goal_y, height + 0.2, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/{0}/base_link".format(robot.name)):
                print "Failed pre-drop"
                return
            x += 0.1

        if not arm.send_goal(dx, goal_y, height + 0.1, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/{0}/base_link".format(robot.name)):
            print "drop"
            return

        # Open gripper
        arm.send_gripper_goal('open')

        x = dx
        while x > 0.3:
            if not arm.send_goal(x, goal_y, height + 0.2, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/{0}/base_link".format(robot.name)):
                print "Failed pre-drop"
                return
            x -= 0.1

        if not arm.send_goal(0.2, goal_y, height + 0.05, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/{0}/base_link".format(robot.name)):
            print "Failed after-drop"
            return

        # Close gripper
        arm.send_gripper_goal('close')

        arm.reset()

class Place(smach.StateMachine):

    def __init__(self, robot, item_to_place_designator, place_pose_designator, arm_designator):
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        with self:
            smach.StateMachine.add('NAVIGATE_TO_PLACE', NavigateToGrasp(robot, place_pose_designator, arm_designator),  # TODO: Navigate to place
                transitions={ 'unreachable' : 'failed',
                              'goal_not_defined' : 'failed',
                              'arrived' : 'PUT'})

            smach.StateMachine.add('PUT', Put(robot, item_to_place_designator, place_pose_designator, arm_designator),
                transitions={'succeeded' :   'done',
                             'failed' :   'failed'})
