#! /usr/bin/env python
import rospy
import smach

from robot_skills.util.kdl_conversions import kdlFrameStampedFromXYZRPY, kdlVectorToPointMsg, FrameStamped
from robot_smach_states.navigation import NavigateToPlace

from robot_smach_states.state import State

from robot_smach_states.util.designators import check_type
from robot_skills.util.entity import Entity
from robot_skills.arms import Arm

class PreparePlace(smach.State):
    def __init__(self, robot, placement_pose, arm):
        """
        Drive the robot back a little and move the designated arm to place the designated item at the designated pose
        :param robot: Robot to execute state with
        :param placement_pose: Designator that resolves to the pose to place at. E.g. an EmptySpotDesignator
        :param arm: Designator -> arm to place with, so Arm that holds entity_to_place, e.g. via ArmHoldingEntityDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Check types or designator resolve types
        check_type(placement_pose, FrameStamped)
        check_type(arm, Arm)

        # Assign member variables
        self._robot = robot
        self._placement_pose_designator = placement_pose
        self._arm_designator = arm

    def execute(self, userdata):

        placement_fs = self._placement_pose_designator.resolve()
        if not placement_fs:
            rospy.logerr("Could not resolve placement_pose")
            return "failed"

        arm = self._arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        # Arm to position in a safe way
        arm.send_joint_trajectory('prepare_place', timeout=0)

        # Torso up (non-blocking)
        # self._robot.torso.high()
        # self._robot.torso.low()
        # When the arm is in the prepare_place configuration, the grippoint is approximately at height torso_pos + 0.6
        # Hence, we want the torso to go to the place height - 0.6
        # Note: this is awefully hardcoded for AMIGO
        torso_goal = placement_fs.frame.p.z() - 0.6
        torso_goal = max(0.09, min(0.4, torso_goal))
        rospy.logwarn("Torso goal before placing: {0}".format(torso_goal))
        self._robot.torso._send_goal(torso_pos=[torso_goal])

        return 'succeeded'

# ----------------------------------------------------------------------------------------------------

class Put(smach.State):

    def __init__(self, robot, item_to_place, placement_pose, arm):
        """
        Drive the robot back a little and move the designated arm to place the designated item at the designated pose
        :param robot: Robot to execute state with
        :param item_to_place: Designator that resolves to the entity to place. e.g EntityByIdDesignator
        :param placement_pose: Designator that resolves to the pose to place at. E.g. an EmptySpotDesignator
        :param arm: Designator -> arm to place with, so Arm that holds entity_to_place, e.g. via ArmHoldingEntityDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Check types or designator resolve types
        check_type(item_to_place, Entity)
        check_type(placement_pose, FrameStamped)
        check_type(arm, Arm)

        # Assign member variables
        self._robot = robot
        self._item_to_place_designator = item_to_place
        self._placement_pose_designator = placement_pose
        self._arm_designator = arm

    def execute(self, userdata): #robot, item_to_place, placement_pose, arm):
        item_to_place = self._item_to_place_designator.resolve()
        if not item_to_place:
            rospy.logerr("Could not resolve item_to_place")
            #return "failed"

        placement_fs = self._placement_pose_designator.resolve()
        if not placement_fs:
            rospy.logerr("Could not resolve placement_pose")
            return "failed"

        arm = self._arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        rospy.loginfo("Placing")

        # placement_pose is a PyKDL.Frame
        place_pose_bl = placement_fs.projectToFrame(self._robot.robot_name+'/base_link', tf_listener=self._robot.tf_listener)

        # Wait for torso and arm to finish their motions
        self._robot.torso.wait_for_motion_done()
        arm.wait_for_motion_done()

        try:
            height = place_pose_bl.frame.p.z()
        except KeyError:
            height = 0.8

        # Pre place
        if not arm.send_goal(place_pose_bl, timeout=10, pre_grasp=False):
            # If we can't place, try a little closer
            place_pose_bl.frame.p.x(place_pose_bl.frame.p.x() - 0.05)

            rospy.loginfo("Retrying preplace")
            if not arm.send_goal(kdlFrameStampedFromXYZRPY(place_pose_bl.frame.p.x(), place_pose_bl.frame.p.y(), height+0.2, 0.0, 0.0, 0.0, frame_id="/{0}/base_link".format(self._robot.robot_name)),
                             timeout=10, pre_grasp=False):
                rospy.logwarn("Cannot pre-place the object")
                arm.cancel_goals()
                return 'failed'

        # Place
        if not arm.send_goal(kdlFrameStampedFromXYZRPY(place_pose_bl.frame.p.x(), place_pose_bl.frame.p.y(), height+0.15, 0.0, 0.0, 0.0, frame_id="/{0}/base_link".format(self._robot.robot_name)),
                             timeout=10, pre_grasp=False):
            rospy.logwarn("Cannot place the object, dropping it...")

        place_entity = arm.occupied_by
        if not place_entity:
            rospy.logerr("Arm not holding an entity to place. This should never happen")
        else:
            self._robot.ed.update_entity(place_entity.id, frame_stamped=placement_fs)
            arm.occupied_by = None

        # Open gripper
        # Since we cannot reliably wait for the gripper, just set this timeout
        arm.send_gripper_goal('open', timeout=2.0)

        arm.occupied_by = None

        # Retract
        arm.send_goal(kdlFrameStampedFromXYZRPY(place_pose_bl.frame.p.x() - 0.1, place_pose_bl.frame.p.y(), place_pose_bl.frame.p.z() + 0.05, 0.0, 0.0, 0.0, frame_id='/'+self._robot.robot_name+'/base_link'),
                             timeout=0.0)

        self._robot.base.force_drive(-0.125, 0, 0, 1.5)

        if not arm.wait_for_motion_done(timeout=5.0):
            rospy.logwarn('Retraction failed')
            arm.cancel_goals()

        # Close gripper
        arm.send_gripper_goal('close', timeout=0.0)

        self._robot.torso.reset()
        arm.reset()
        arm.wait_for_motion_done()

        return 'succeeded'

class Place(smach.StateMachine):

    def __init__(self, robot, item_to_place, place_pose, arm):
        """
        Drive the robot to be close to the designated place_pose and move the designated arm to place the designated item there
        :param robot: Robot to execute state with
        :param item_to_place: Designator that resolves to the entity to place. e.g EntityByIdDesignator
        :param place_pose: Designator that resolves to the pose to place at. E.g. an EmptySpotDesignator
        :param arm: Designator -> arm to place with, so Arm that holds entity_to_place, e.g. via ArmHoldingEntityDesignator
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        #Check types or designator resolve types
        assert(item_to_place.resolve_type == Entity or type(item_to_place) == Entity)
        assert(place_pose.resolve_type == FrameStamped or type(place_pose) == FrameStamped)
        assert(arm.resolve_type == Arm or type(arm) == Arm)

        with self:
            smach.StateMachine.add('PREPARE_PLACE', PreparePlace(robot, place_pose, arm),
                transitions={ 'succeeded' : 'NAVIGATE_TO_PLACE',
                              'failed' : 'failed'})

            smach.StateMachine.add('NAVIGATE_TO_PLACE', NavigateToPlace(robot, place_pose, arm),
                transitions={ 'unreachable' : 'failed',
                              'goal_not_defined' : 'failed',
                              'arrived' : 'PUT'})

            smach.StateMachine.add('PUT', Put(robot, item_to_place, place_pose, arm),
                transitions={'succeeded' :   'done',
                             'failed' :   'failed'})
