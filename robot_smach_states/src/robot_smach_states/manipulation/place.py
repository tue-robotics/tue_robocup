# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills.arms import PublicArm
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import kdl_frame_stamped_from_XYZRPY, FrameStamped
from robot_smach_states.navigation import NavigateToPlace
from robot_smach_states.world_model import Inspect
from robot_smach_states.utility import LockDesignator
from robot_smach_states.util.designators.utility import LockingDesignator
from .place_designator import EmptySpotDesignator
from robot_smach_states.util.designators import check_type


class PreparePlace(smach.State):
    def __init__(self, robot, arm):
        """
        Drive the robot back a little and move the designated arm to place the designated item at the designated pose

        :param robot: Robot to execute state with
        :param arm: Designator -> arm to place with, so Arm that holds entity_to_place, e.g. via
        ArmHoldingEntityDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Check types or designator resolve types
        check_type(arm, PublicArm)

        # Assign member variables
        self._robot = robot
        self._arm_designator = arm

    def execute(self, userdata=None):

        arm = self._arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        # Torso up (non-blocking)
        self._robot.torso.reset()

        # Arm to position in a safe way
        arm.send_joint_trajectory('prepare_place', timeout=0)
        arm.wait_for_motion_done()

        # When the arm is in the prepare_place configuration, the grippoint is approximately at height torso_pos + 0.6
        # Hence, we want the torso to go to the place height - 0.6
        # Note: this is awefully hardcoded for AMIGO
        # Sending it to 'high' seems to work much better...
        # torso_goal = placement_fs.frame.p.z() - 0.6
        # torso_goal = max(0.09, min(0.4, torso_goal))
        # rospy.logwarn("Torso goal before placing: {0}".format(torso_goal))
        # self._robot.torso._send_goal(torso_pos=[torso_goal])

        return 'succeeded'

# ----------------------------------------------------------------------------------------------------


class Put(smach.State):

    def __init__(self, robot, item_to_place, placement_pose, arm):
        """
        Drive the robot back a little and move the designated arm to place the designated item at the designated pose

        :param robot: Robot to execute state with
        :param item_to_place: Designator that resolves to the entity to place. e.g EntityByIdDesignator
        :param placement_pose: Designator that resolves to the pose to place at. E.g. an EmptySpotDesignator
        :param arm: Designator -> arm to place with, so Arm that holds entity_to_place, e.g. via
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

    def execute(self, userdata=None):

        item_to_place = self._item_to_place_designator.resolve()
        if not item_to_place:
            rospy.logerr("Could not resolve item_to_place")
            # return "failed"

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
        place_pose_bl = placement_fs.projectToFrame(self._robot.robot_name+'/base_link',
                                                    tf_listener=self._robot.tf_listener)

        # Wait for torso and arm to finish their motions
        self._robot.torso.wait_for_motion_done()
        arm.wait_for_motion_done()

        try:
            height = place_pose_bl.frame.p.z()
        except KeyError:
            height = 0.8

        # Pre place
        if not arm.send_goal(kdl_frame_stamped_from_XYZRPY(place_pose_bl.frame.p.x(),
                                                           place_pose_bl.frame.p.y(),
                                                           height+0.15, 0.0, 0.0, 0.0,
                                                           frame_id="/{0}/base_link".format(self._robot.robot_name)),
                             timeout=10,
                             pre_grasp=True):
            # If we can't place, try a little closer
            place_pose_bl.frame.p.x(place_pose_bl.frame.p.x() - 0.025)

            rospy.loginfo("Retrying preplace")
            if not arm.send_goal(kdl_frame_stamped_from_XYZRPY(place_pose_bl.frame.p.x(),
                                                               place_pose_bl.frame.p.y(),
                                                               height+0.15, 0.0, 0.0, 0.0,
                                                               frame_id="/{0}/base_link".format(self._robot.robot_name)
                                                               ), timeout=10, pre_grasp=True):
                rospy.logwarn("Cannot pre-place the object")
                arm.cancel_goals()
                return 'failed'

        # Place
        place_pose_bl = placement_fs.projectToFrame(self._robot.robot_name+'/base_link',
                                                    tf_listener=self._robot.tf_listener)
        if not arm.send_goal(kdl_frame_stamped_from_XYZRPY(place_pose_bl.frame.p.x(),
                                                           place_pose_bl.frame.p.y(),
                                                           height+0.1, 0.0, 0.0, 0.0,
                                                           frame_id="/{0}/base_link".format(self._robot.robot_name)),
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
        place_pose_bl = placement_fs.projectToFrame(self._robot.robot_name+'/base_link',
                                                    tf_listener=self._robot.tf_listener)
        arm.send_goal(kdl_frame_stamped_from_XYZRPY(place_pose_bl.frame.p.x() - 0.1,
                                                    place_pose_bl.frame.p.y(),
                                                    place_pose_bl.frame.p.z() + 0.15, 0.0, 0.0, 0.0,
                                                    frame_id='/'+self._robot.robot_name+'/base_link'),
                      timeout=0.0)

        arm.wait_for_motion_done()
        self._robot.base.force_drive(-0.125, 0, 0, 1.5)

        if not arm.wait_for_motion_done(timeout=5.0):
            rospy.logwarn('Retraction failed')
            arm.cancel_goals()

        # Close gripper
        arm.send_gripper_goal('close', timeout=0.0)

        arm.reset()
        arm.wait_for_motion_done()
        self._robot.torso.reset()
        self._robot.torso.wait_for_motion_done()

        return 'succeeded'


class Place(smach.StateMachine):

    def __init__(self, robot, item_to_place, place_pose, arm, place_volume=None, update_supporting_entity=False):
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
        :param update_supporting_entity: (optional) bool to indicate whether the supporting entity should be updated.
            This can only be used if the supporting entity is supplied, case 2 or 3 mentioned under item_to_place
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types
        assert(item_to_place.resolve_type == Entity or type(item_to_place) == Entity)
        assert(arm.resolve_type == PublicArm or type(arm) == PublicArm)
        #assert(place_volume.resolve_type == str or (type(place_volume) == str))

        # parse place volume
        if place_volume is not None:
            if isinstance(place_volume, str):
                place_area = place_volume
            elif place_volume.resolve_type == str:
                place_area = place_volume.resolve()
            else:
                raise AssertionError("Cannot place in {}".format(place_volume))

        # Case 3
        if isinstance(place_pose, str):
            furniture_designator = EdEntityDesignator(robot=robot, id=place_pose)
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
                                       transitions={'done': 'PREPARE_PLACE',
                                                    'failed': 'failed'}
                                       )

            smach.StateMachine.add('PREPARE_PLACE', PreparePlace(robot, arm),
                                   transitions={'succeeded': 'LOCK_DESIGNATOR',
                                                'failed': 'failed'})

            smach.StateMachine.add('LOCK_DESIGNATOR', LockDesignator(locking_place_designator),
                                   transitions={'locked': 'NAVIGATE_TO_PLACE'})

            smach.StateMachine.add('NAVIGATE_TO_PLACE', NavigateToPlace(robot, locking_place_designator, arm),
                                   transitions={'unreachable': 'failed',
                                                'goal_not_defined': 'failed',
                                                'arrived': 'PUT'})

            smach.StateMachine.add('PUT', Put(robot, item_to_place, locking_place_designator, arm),
                                   transitions={'succeeded': 'done',
                                                'failed': 'failed'})


if __name__ == "__main__":
    from robot_skills import arms
    from robot_skills import get_robot_from_argv
    from robot_smach_states.util.designators import EdEntityDesignator, ArmDesignator

    rospy.init_node('state_machine')

    robot = get_robot_from_argv(index=1)

    robot.ed.update_entity(id="bla")
    place_entity = EdEntityDesignator(robot, id="bla")
    arm = ArmDesignator(robot, arm_properties={"required_trajectories": ["prepare_place"],
                                               "required_grasping_types": [arms.GripperTypes.GRASPING]})

    sm = Place(robot=robot, item_to_place=place_entity, place_pose='dinner_table', arm=arm, place_volume='on_top_of')
    print(sm.execute())
