#! /usr/bin/env python
# ROS

import smach
import rospy

import PyKDL as kdl

from robot_skills.util.kdl_conversions import VectorStamped


class HeroHmiPose(smach.State):
    """
    State to ensure that hero's arm is out of the way so that the robot
    may move his torso up.
    :param robot: Robot to execute state with
    """
    def __init__(self, robot, rotation=1.57, rotational_speed=1):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._robot = robot
        self._rotation = rotation
        self._rot_speed = rotational_speed

    def execute(self, _):
        arm = self._robot.get_arm()

        # Get position to look at after turning. Transform the position to map frame since turning will move base_link
        goal = VectorStamped(1.0, 0.0, 1.6, frame_id="/" + self._robot.robot_name + "/base_link")
        tf_goal = goal.projectToFrame('/map', self._robot.tf_listener)

        rotation_duration = self._rotation / self._rot_speed
        if arm.has_joint_goal('arm_out_of_way'):
            arm.send_joint_goal('arm_out_of_way', 0.0)
            self._robot.base.force_drive(0, 0, self._rot_speed, rotation_duration)
        self._robot.head.look_at_point(tf_goal)
        arm.wait_for_motion_done()
        self._robot.head.wait_for_motion_done()
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
        self._robot.head.wait_for_motion_done()
        return "succeeded"


class HeroInspectPose(smach.State):
    """
    State to ensure that hero's arm is out of the way so that the robot
    may move his torso up.
    :param robot: Robot to execute state with
    """
    def __init__(self, robot, entity, volume=None, rotation=1.57, rotational_speed=1):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._robot = robot
        self._rotation = rotation
        self._rot_speed = rotational_speed
        self._entity = entity
        self._volume = volume

    def execute(self, _):

        # Determine the height of the head target
        # Start with a default
        entity = self._entity.resolve()
        volume = self._volume

        # Check if we have areas: use these
        if volume in entity.volumes:
            search_volume = entity.volumes[volume]
            x_obj = 0.5 * (search_volume.min_corner.x() + search_volume.max_corner.x())
            y_obj = 0.5 * (search_volume.min_corner.y() + search_volume.max_corner.y())
            z_obj = search_volume.min_corner.z()
            lookat_pos_map = entity.pose.frame * kdl.Vector(x_obj, y_obj, z_obj)
            z = lookat_pos_map.z()
        else:
            # Look at the top of the entity to inspect
            pos = entity.pose.frame.p
            z = pos.z() + entity.shape.z_max

        # calculate the arm_lift_link which must be sent
        z_over = 0.3  # m
        z_hh = 0.9  # height of the robots head at z_arm=0
        torso_to_arm_ratio = 2  # motion of arm/motion of torso
        z_head = z + z_over

        # check whether moving the arm is necessary
        if z_head < 1.2:
            rospy.loginfo('Entity is low enough. we dont need to move the arm')
            return "succeeded"

        # saturate the arm lift goal
        z_arm = (z_head-z_hh)*torso_to_arm_ratio
        if z_arm > 0.69:
            z_arm = 0.69
            rospy.logwarn('Warning: looking at excessively high surface')
        elif z_arm < 0.0:
            z_arm = 0.0
            rospy.logwarn('Surface is low enough, we dont need to move the arm.')

        arm = self._robot.get_arm()

        rotation_duration = self._rotation / self._rot_speed
        if arm.has_joint_goal('arm_out_of_way'):
            pose = arm._arm.default_configurations['arm_out_of_way']
            pose[0] = z_arm
            arm._arm._send_joint_trajectory([pose])
        else:
            rospy.logwarn('Warning: robot does not have an "arm_out_of_way" joint goal')
            return "failed"
        self._robot.base.force_drive(0, 0, self._rot_speed, rotation_duration)
        arm.wait_for_motion_done()
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
            if robot.robot_name == 'hero':
                smach.StateMachine.add('HERO_HMI_POSE', HeroHmiPose(robot),
                                       transitions={'succeeded': 'done',
                                                    'failed': 'failed'})
            else:
                smach.StateMachine.add('HMI_POSE', HmiPose(robot),
                                       transitions={'succeeded': 'done',
                                                    'failed': 'failed'})


class RiseForInspect(smach.StateMachine):
    """
    Get the robot in a pose from which it can perform an inspect. This is more complicated for HERO compared to
    AMIGO and SERGIO.
    :param robot: Robot to execute state with
    """

    def __init__(self, robot, entity, volume=None):
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        with self:
            if robot.robot_name == 'hero':
                smach.StateMachine.add('HERO_INSPECT_POSE', HeroInspectPose(robot, entity, volume),
                                       transitions={'succeeded': 'done',
                                                    'failed': 'failed'})
            else:
                smach.StateMachine.add('INSPECT_POSE', HmiPose(robot),
                                       transitions={'succeeded': 'done',
                                                    'failed': 'failed'})
