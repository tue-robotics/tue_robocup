#! /usr/bin/env python
# ROS
import rospy
import smach

from robot_skills.util.kdl_conversions import VectorStamped


class HeroHMIPose(smach.State):
    """
    State to ensure that hero's arm is out of the way so that the robot
    may move his torso up.
    :param robot: Robot to execute state with
    """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._robot = robot

    def execute(self, _):
        rotation = 1.57
        rot_speed = 1
        arm = self._robot.get_arm()

        #get position to look at
        goal = VectorStamped(1.0, 0.0, 1.6, frame_id="/" + self._robot.robot_name + "/base_link")
        tf_goal = goal.projectToFrame('/map', self._robot.tf_listener)

        if arm.has_joint_goal('arm_out_of_way'):
            arm.send_joint_goal('arm_out_of_way', 0.0)
            self._robot.base.force_drive(0, 0, rot_speed, rotation / rot_speed)
        self._robot.head.look_at_point(tf_goal)
        arm.wait_for_motion_done()
        return "succeeded"


class RiseForHMI(smach.StateMachine):
    """
    Drive the robot to be close to the designated place_pose and move the designated arm to place the designated
    item there
    :param robot: Robot to execute state with
    :param item_to_place: Designator that resolves to the entity to place. e.g EntityByIdDesignator
    :param place_pose: The place pose can be one of three things:
        1: Designator that resolves to the pose to place at. E.g. an EmptySpotDesignator
        2: EdEntityDesignator resolving to an object on which the robot should place something
        3: A string identifying an object on which the robot should place something
    :param arm: Designator -> arm to place with, so Arm that holds entity_to_place, e.g. via
    ArmHoldingEntityDesignator
    :param place_volume (optional) string identifying the volume where to place the object, e.g., 'on_top_of',
    shelf3'
    :param update_supporting_entity (optional) bool to indicate whether the supporting entity should be updated.
    This can only be used if the supporting entity is supplied, case 2 or 3 mentioned under item_to_place
    """

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        with self:
            smach.StateMachine.add('ARM_OUT_OF_WAY', HeroHMIPose(robot),
                                   transitions={'succeeded': 'done',
                                                'failed': 'failed'})
