from __future__ import absolute_import, print_function
from typing import Union, Optional

# ROS
from pykdl_ros import FrameStamped
import rospy
import smach

# TU/e Robotics
from ed.entity import Entity
from robot_skills.arm.arms import PublicArm, GripperTypes
from robot_skills.robot import Robot
from .place_designator import EmptySpotDesignator
from ..navigation.navigate_to_place import NavigateToPlace
from ..utility import LockDesignator, ResolveArm, UnlockDesignator, check_arm_requirements
from ..util.designators import check_type
from ..util.designators.utility import LockingDesignator
from robot_smach_states.world_model.world_model import Inspect
from ..util.designators.core import Designator


class Put(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING], }

    def __init__(self, robot: Robot, item_to_place: Designator[Entity], placement_pose: Designator[FrameStamped], arm: Designator[PublicArm]) -> None:
        """
        Drive the robot back a little and move the designated arm to place the designated item at the designated pose

        :param robot: Robot to execute state with
        :param item_to_place: Designator that resolves to the entity to place. e.g EntityByIdDesignator
        :param placement_pose: Designator that resolves to the pose to place at. E.g. an EmptySpotDesignator
        :param locked arm: Locked arm designator -> arm to place with, so Arm that holds entity_to_place, e.g. via
            ArmHoldingEntityDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Check types or designator resolve types
        check_type(item_to_place, Entity)
        check_type(placement_pose, FrameStamped)
        check_type(arm, PublicArm)

        # Assign member variables
        self._robot = robot
        self._item_to_place_designator = item_to_place
        self._placement_pose_designator = placement_pose
        self._arm_designator = arm
        self._center_height = 0.1  # height offset w.r.t. place_pose
        self._preplace_offset = 0.05  # height offset for preplace (in addition to center_height)

    def execute(self, userdata=None):

        item_to_place = self._item_to_place_designator.resolve()
        if not item_to_place:
            rospy.logerr("Could not resolve item_to_place")
            # return "failed"

        placement_fs = self._placement_pose_designator.resolve()
        if not placement_fs:
            rospy.logerr("Could not resolve placement_pose")
            return "failed"
        placement_fs.header.stamp = rospy.Time(0)  # always query the latest pose info

        arm = self._arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        rospy.loginfo("Placing")

        # placement_pose is a PyKDL.Frame
        place_pose_bl = self._robot.tf_buffer.transform(placement_fs, self._robot.base_link_frame,
                                                        timeout=rospy.Duration(1.0))

        # Wait for arm to finish their motions
        arm.wait_for_motion_done()

        try:
            height = place_pose_bl.frame.p.z()
        except KeyError:
            height = 0.8

        # Pre place
        if not arm.send_goal(FrameStamped.from_xyz_rpy(place_pose_bl.frame.p.x(), place_pose_bl.frame.p.y(),
                                                       height + self._center_height + self._preplace_offset, 0, 0, 0, rospy.Time(0),
                                                       frame_id=self._robot.base_link_frame),
                             timeout=10,
                             pre_grasp=True):
            # If we can't place, try a little closer
            place_pose_bl.frame.p.x(place_pose_bl.frame.p.x() - 0.025)

            rospy.loginfo("Retrying preplace")
            if not arm.send_goal(FrameStamped.from_xyz_rpy(place_pose_bl.frame.p.x(), place_pose_bl.frame.p.y(),
                                                           height + self._center_height + self._preplace_offset, 0, 0, 0, rospy.Time(0),
                                                           frame_id=self._robot.base_link_frame),
                                 timeout=10, pre_grasp=True):
                rospy.logwarn("Cannot pre-place the object")
                arm.cancel_goals()
                return 'failed'

        # Place
        place_pose_bl = self._robot.tf_buffer.transform(placement_fs, self._robot.base_link_frame, timeout=rospy.Duration(1.0))
        actual_place_pose_bl = FrameStamped.from_xyz_rpy(place_pose_bl.frame.p.x(), place_pose_bl.frame.p.y(),
                                                         height + self._center_height, 0, 0, 0, rospy.Time(0),
                                                         frame_id=self._robot.base_link_frame)
        if not arm.send_goal(actual_place_pose_bl, timeout=10, pre_grasp=False):
            rospy.logwarn("Cannot place the object, dropping it...")

        place_entity = arm.gripper.occupied_by
        if not place_entity:
            rospy.logerr("Arm not holding an entity to place. This should never happen")
        else:
            place_pose_map = self._robot.tf_buffer.transform(actual_place_pose_bl, "map", timeout=rospy.Duration(1.0))
            new_entity_pose = FrameStamped(place_pose_map.frame * place_entity.pose.frame,
                                           place_pose_bl.header.stamp,
                                           "map")
            self._robot.ed.update_entity(place_entity.uuid, frame_stamped=new_entity_pose)
            arm.gripper.occupied_by = None

        # Open gripper
        # Since we cannot reliably wait for the gripper, just set this timeout
        arm.gripper.send_goal('open', timeout=2.0)

        arm.gripper.occupied_by = None

        # Retract
        place_pose_bl = self._robot.tf_buffer.transform(placement_fs, self._robot.base_link_frame, timeout=rospy.Duration(1.0))
        arm.send_goal(FrameStamped.from_xyz_rpy(place_pose_bl.frame.p.x() - 0.1,
                                                place_pose_bl.frame.p.y(),
                                                place_pose_bl.frame.p.z() + self._center_height + self._preplace_offset,
                                                0, 0, 0,
                                                rospy.Time(0),
                                                frame_id=self._robot.base_link_frame),
                      timeout=0.0)

        # Wait for arm to finish their motions
        arm.wait_for_motion_done()

        # Move back
        self._robot.base.force_drive(-0.125, 0, 0, 1.5, ax=0.5)

        if not arm.wait_for_motion_done(timeout=5.0):
            rospy.logwarn('Retraction failed')
            arm.cancel_goals()

        arm.reset()
        arm.wait_for_motion_done()

        return 'succeeded'


class Place(smach.StateMachine):

    def __init__(self, robot: Robot, item_to_place: Designator[Entity], place_pose: Union[Designator[str], Designator[Entity], str], arm: Designator[PublicArm],
                 place_volume: Optional[str] = None) -> None:
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
        :param place_volume: (optional) string identifying the volume where to place the object, e.g., 'on_top_of',
            'shelf3'
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types
        check_type(item_to_place, Entity)
        check_type(arm, PublicArm)

        # parse place volume
        if place_volume is not None:
            if isinstance(place_volume, str):
                place_area = place_volume
            elif place_volume.resolve_type == str:
                place_area = place_volume.resolve()
            else:
                raise AssertionError("Cannot place in {}".format(place_volume))
        else:
            place_area = None

        # Case 3
        if isinstance(place_pose, str):
            furniture_designator = EdEntityDesignator(robot=robot, uuid=place_pose)
            place_designator = EmptySpotDesignator(robot=robot, place_location_designator=furniture_designator,
                                                   arm_designator=arm, area=place_area)
        # Case 1
        elif place_pose.resolve_type == FrameStamped or type(place_pose) == FrameStamped:
            furniture_designator = None
            place_designator = place_pose
        # Case 2
        elif place_pose.resolve_type == Entity:
            furniture_designator = place_pose
            place_designator = EmptySpotDesignator(robot=robot, place_location_designator=furniture_designator,
                                                   arm_designator=arm, area=place_area)
        else:
            raise AssertionError("Cannot place on {}".format(place_pose))

        locking_place_designator = LockingDesignator(place_designator)

        with self:

            if furniture_designator is not None:
                smach.StateMachine.add('INSPECT',
                                       Inspect(robot, furniture_designator, navigation_area="in_front_of"),
                                       transitions={'done': 'RESOLVE_ARM',
                                                    'failed': 'failed'}
                                       )
            smach.StateMachine.add('RESOLVE_ARM', ResolveArm(arm, self),
                                   transitions={'succeeded': 'LOCK_DESIGNATOR',
                                                'failed': 'failed'})

            smach.StateMachine.add('LOCK_DESIGNATOR', LockDesignator(locking_place_designator),
                                   transitions={'locked': 'NAVIGATE_TO_PLACE'})

            smach.StateMachine.add('NAVIGATE_TO_PLACE', NavigateToPlace(robot, locking_place_designator, arm),
                                   transitions={'unreachable': 'failed',
                                                'goal_not_defined': 'failed',
                                                'arrived': 'PUT'})

            smach.StateMachine.add('PUT', Put(robot, item_to_place, locking_place_designator, arm),
                                   transitions={'succeeded': 'UNLOCK_DESIGNATOR_SUCCESS',
                                                'failed': 'UNLOCK_DESIGNATOR_FAILED'})

            # This is needed to be able to reuse the same instance of this state machine
            smach.StateMachine.add('UNLOCK_DESIGNATOR_SUCCESS', UnlockDesignator(locking_place_designator),
                                   transitions={'unlocked': 'done'})

            smach.StateMachine.add('UNLOCK_DESIGNATOR_FAILED', UnlockDesignator(locking_place_designator),
                                   transitions={'unlocked': 'failed'})

            check_arm_requirements(self, robot)


if __name__ == "__main__":
    from robot_skills.arm import arms
    from robot_skills import get_robot_from_argv
    from robot_smach_states.util.designators import EdEntityDesignator, ArmDesignator

    rospy.init_node('place_test')

    robot = get_robot_from_argv(index=1)

    robot.ed.update_entity(uuid="bla")
    place_entity = EdEntityDesignator(robot, uuid="bla")
    arm = ArmDesignator(robot, arm_properties={"required_trajectories": ["prepare_place"],
                                               "required_gripper_types": [arms.GripperTypes.GRASPING]})

    sm = Place(robot=robot, item_to_place=place_entity, place_pose='dinner_table', arm=arm.lockable(),
               place_volume='on_top_of')
    print(sm.execute())
