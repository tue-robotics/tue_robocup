#! /usr/bin/env python
import rospy
import smach

import robot_skills.util.transformations as transformations
from robot_smach_states.navigation import NavigateToPlace

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

        if arm == robot.arms["left"]:
            goal_y = 0.2
        else:
            goal_y = -0.2

        try:
            height = placement_in_baselink.z
        except KeyError:
            height = 0.8

        dx = 0.5

        x = 0.2
        while x <= dx:
            if not arm.send_goal(x, goal_y, height + 0.2, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/{0}/base_link".format(robot.robot_name)):
                print "Failed pre-drop"
                return 'failed'
            x += 0.1

        if not arm.send_goal(dx, goal_y, height + 0.1, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/{0}/base_link".format(robot.robot_name)):
            print "drop"
            return 'failed'

        # Open gripper
        arm.send_gripper_goal('open')

        arm.occupied_by = None

        x = dx
        while x > 0.3:
            if not arm.send_goal(x, goal_y, height + 0.2, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/{0}/base_link".format(robot.robot_name)):
                print "Failed pre-drop"
                return 'failed'
            x -= 0.1

        if not arm.send_goal(0.2, goal_y, height + 0.05, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/{0}/base_link".format(robot.robot_name)):
            print "Failed after-drop"
            return 'failed'

        # Close gripper
        arm.send_gripper_goal('close')

        arm.reset()
        
        return 'succeeded'

class Place(smach.StateMachine):

    def __init__(self, robot, item_to_place, place_pose, arm):
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        with self:
            smach.StateMachine.add('NAVIGATE_TO_PLACE', NavigateToPlace(robot, place_pose, arm),  # TODO: Navigate to place
                transitions={ 'unreachable' : 'failed',
                              'goal_not_defined' : 'failed',
                              'arrived' : 'PUT'})

            smach.StateMachine.add('PUT', Put(robot, item_to_place, place_pose, arm),
                transitions={'succeeded' :   'done',
                             'failed' :   'failed'})
