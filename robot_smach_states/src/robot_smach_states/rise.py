#! /usr/bin/env python
# ROS

import smach

from robot_skills.util.kdl_conversions import VectorStamped


class HeroHmiPose(smach.State):
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

class HmiPose(smach.State):
    """
    State to puts the robot in a position for hmi
    :param robot: Robot to execute state with
    """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._robot = robot

    def execute(self, _):
        self._robot.head.look_at_standing_person()
        return "succeeded"


class RiseForHMI(smach.StateMachine):
    """
    Get the robot in a nice pose for human machine interaction. This is more complicated for HERO compared to
    AMIGO and SERGIO.
    :param robot: Robot to execute state with
    """

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        with self:
            if robot.robot_name == 'hero2':
                smach.StateMachine.add('HERO_HMI_POSE', HeroHmiPose(robot),
                                       transitions={'succeeded': 'done',
                                                    'failed': 'failed'})
            else:
                smach.StateMachine.add('HMI_POSE', HmiPose(robot),
                                       transitions={'succeeded': 'done',
                                                    'failed': 'failed'})
