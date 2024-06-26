from __future__ import absolute_import

# ROS
import PyKDL as kdl
from pykdl_ros import VectorStamped, FrameStamped
import rospy
import smach
import tf2_ros

# TU/e Robotics
from ed.entity import Entity
from robot_skills.robot import Robot
from robot_skills.arm.arms import PublicArm, GripperTypes
from ..utility import check_arm_requirements, ResolveArm
from ..util.designators import check_type
from ..navigation.navigate_to_grasp import NavigateToGrasp
from ..manipulation.grasp_point_determination import GraspPointDeterminant
from ..util.designators.arm import ArmDesignator
from ..util.designators.core import Designator


class PrepareEdGrasp(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING],
                               "required_trajectories": ["prepare_grasp"], }

    def __init__(self, robot: Robot, arm: ArmDesignator, grab_entity: Designator) -> None:
        """
        Set the arm in the appropriate position before actually grabbing

        :param robot: robot to execute state with
        :param arm: Designator that resolves to the arm to grasp with
        :param grab_entity: Designator that resolves to the entity to grab. e.g EntityByIdDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Assign member variables
        self.robot = robot
        self.arm_designator = arm
        self.grab_entity_designator = grab_entity

        check_type(grab_entity, Entity)

    def execute(self, userdata=None) -> str:
        entity = self.grab_entity_designator.resolve()
        if not entity:
            rospy.logerr("Could not resolve grab_entity")
            return "failed"

        self.robot.move_to_inspect_pose(entity._pose.p)
        entity_pose_vs = VectorStamped.from_framestamped(entity.pose)
        self.robot.head.look_at_point(entity_pose_vs, timeout=0.0)
        self.robot.head.wait_for_motion_done()
        segm_res = self.robot.ed.update_kinect("%s" % entity.uuid)

        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        # Torso up (non-blocking)
        self.robot.torso.reset()

        # Arm to position in a safe way
        self.robot.move_to_pregrasp_pose(arm, entity._pose.p)
        arm.wait_for_motion_done()

        # Open gripper
        arm.gripper.send_goal('open', timeout=0.0)
        arm.wait_for_motion_done()

        # Make sure the head looks at the entity
        self.robot.head.look_at_point(entity_pose_vs, timeout=0.0)
        self.robot.head.wait_for_motion_done()
        return 'succeeded'


class PickUp(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING],
                               "required_goals": ["carrying_pose"], }

    def __init__(self, robot: Robot, arm: ArmDesignator, grab_entity: Designator,
                 check_occupancy: bool = False) -> None:
        """
        Pick up an item given an arm and an entity to be picked up

        :param robot: robot to execute this state with
        :param arm: Designator that resolves to the arm to grasp with
        :param grab_entity: Designator that resolves to the entity to grab. e.g EntityByIdDesignator
        :param check_occupancy: Indicates whether to check if gripper is occupied
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Assign member variables
        self.robot = robot
        self.arm_designator = arm
        check_type(grab_entity, Entity)
        self.grab_entity_designator = grab_entity
        self._gpd = GraspPointDeterminant(robot)
        self._check_occupancy = check_occupancy

        assert self.robot.get_arm(**self.REQUIRED_ARM_PROPERTIES) is not None,\
            "None of the available arms meets all this class's requirements: {}".format(self.REQUIRED_ARM_PROPERTIES)

    def execute(self, userdata=None) -> str:

        grab_entity = self.grab_entity_designator.resolve()
        if not grab_entity:
            rospy.logerr("Could not resolve grab_entity")
            return "failed"

        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        goal_map = VectorStamped.from_xyz(0, 0, 0, rospy.Time(), frame_id=grab_entity.uuid)

        try:
            # Transform to base link frame
            goal_bl = self.robot.tf_buffer.transform(goal_map, self.robot.base_link_frame, timeout=rospy.Duration(1.0))
            if goal_bl is None:
                rospy.logerr('Transformation of goal to base failed')
                return 'failed'
        except tf2_ros.TransformException as tfe:
            rospy.logerr('Transformation of goal to base failed: {0}'.format(tfe))
            return 'failed'

        # Make sure the torso and the arm are done
        self.robot.torso.wait_for_motion_done(cancel=True)
        arm.wait_for_motion_done(cancel=True)

        # This is needed because the head is not entirely still when the
        # look_at_point function finishes
        rospy.sleep(rospy.Duration(0.5))

        # Resolve the entity again because we want the latest pose
        updated_grab_entity = self.grab_entity_designator.resolve()

        rospy.loginfo("ID to update: {0}".format(grab_entity.uuid))
        if not updated_grab_entity:
            rospy.logerr("Could not resolve the updated grab_entity, "
                         "this should not happen [CHECK WHY THIS IS HAPPENING]")
            grab_entity = self.associate(original_entity=grab_entity)
        else:
            rospy.loginfo("Updated pose of entity (dx, dy, dz) : (%f, %f, %f)" %
                          (updated_grab_entity.pose.frame.p.x() - grab_entity.pose.frame.p.x(),
                           updated_grab_entity.pose.frame.p.y() - grab_entity.pose.frame.p.y(),
                           updated_grab_entity.pose.frame.p.z() - grab_entity.pose.frame.p.z()))
            grab_entity = updated_grab_entity

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Grasp point determination
        grasp_framestamped = self._gpd.get_grasp_pose(grab_entity, arm)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        goal_map = VectorStamped.from_xyz(0, 0, 0, rospy.Time(), frame_id=grab_entity.uuid)

        # In case grasp point determination didn't work
        if not grasp_framestamped:
            goal_bl = self.robot.tf_buffer.transform(goal_map, self.robot.base_link_frame, timeout=rospy.Duration(1.0))
            if goal_bl is None:
                return 'failed'
            else:
                return 'failed'
        else:
            # We do have a grasp pose, given as a kdl frame in map
            try:
                self.robot.tf_buffer.can_transform("map", self.robot.base_link_frame, rospy.Time(), rospy.Duration(10))
                # Transform to base link frame
                goal_bl = self.robot.tf_buffer.transform(grasp_framestamped, self.robot.base_link_frame, timeout=rospy.Duration(1.0))
                if goal_bl is None:
                    return 'failed'
            except tf2_ros.TransformException as tfe:
                rospy.logerr('Transformation of goal to base failed: {0}'.format(tfe))
                return 'failed'

        # Grasp
        rospy.loginfo('Start grasping')
        if not arm.send_goal(goal_bl, timeout=20, pre_grasp=True, allowed_touch_objects=[grab_entity.uuid]):
            self.robot.speech.speak('I am sorry but I cannot move my arm to the object position', block=False)
            rospy.logerr('Grasp failed')
            arm.reset()
            arm.gripper.send_goal('close', timeout=0.0)
            return 'failed'

        # Close gripper
        arm.gripper.send_goal('close')
        # Define the pose of the object relative to the gripper (which made contact at grasp_framestamped)
        pose_in_hand = FrameStamped(grasp_framestamped.frame.Inverse() * grab_entity.pose.frame,
                                    grasp_framestamped.header.stamp,
                                    "map" # all entities in ED must be defined with respect to map. We will ignore this property
                                    )
        grab_entity.pose = pose_in_hand
        arm.gripper.occupied_by = grab_entity

        # Retract
        goal_bl = self.robot.tf_buffer.transform(grasp_framestamped, self.robot.base_link_frame, timeout=rospy.Duration(1.0))
        rospy.loginfo('Start retracting')
        roll = 0.0

        goal_bl.frame.p.x(goal_bl.frame.p.x() - 0.1)  # Retract 10 cm
        goal_bl.frame.p.z(goal_bl.frame.p.z() + 0.1)  # Go 10 cm higher
        goal_bl.frame.M = kdl.Rotation.RPY(roll, 0.0, 0.0)  # Update the roll
        rospy.loginfo("Start retract")
        if not arm.send_goal(goal_bl, timeout=0.0, allowed_touch_objects=[grab_entity.uuid]):
            rospy.logerr('Failed retract')
        arm.wait_for_motion_done()
        self.robot.base.force_drive(-0.125, 0, 0, 2.0)

        # Define the pose of the object relative to the gripper (which made contact at grasp_framestamped)
        pose_in_hand = FrameStamped(grasp_framestamped.frame.Inverse() * grab_entity.pose.frame,
                                    grasp_framestamped.header.stamp,
                                    "map" # all entities in ED must be defined with respect to map. We will ignore this property
                                    )
        self.robot.ed.update_entity(uuid=grab_entity.uuid, frame_stamped=pose_in_hand)
        # Remove pose from ED as we are holding the object in the gripper
        self.robot.ed.update_entity(uuid=grab_entity.uuid, remove_pose=True)

        # Carrying pose
        arm.send_joint_goal('carrying_pose', timeout=0.0)

        arm.wait_for_motion_done(cancel=True)

        result = 'succeeded'
        if self._check_occupancy and hasattr(arm.gripper, 'grasp_sensor'):
            # Check if the object is present in the gripper
            if arm.gripper.grasp_sensor.object_in_gripper_measurement.is_empty:
                # If state is empty, grasp has failed
                result = "failed"
                rospy.logerr("Gripper is not holding an object")
                self.robot.speech.speak("Whoops, something went terribly wrong")
                arm.gripper.occupied_by = None  # Set the object the arm is holding to None
            else:
                # State is holding, grasp succeeded.
                # If unknown: sensor not there, assume gripper is holding and hope for the best
                result = "succeeded"
                if arm.gripper.grasp_sensor.object_in_gripper_measurement.is_unknown:
                    rospy.logwarn("GripperMeasurement unknown")

        # Reset head
        self.robot.head.cancel_goal()

        return result

    def associate(self, original_entity: Entity) -> Entity:
        """
        Tries to associate the original entity with one of the entities in the world model. This is useful if
        after an update, the original entity is no longer present in the world model. If no good map can be found,
        the original entity will be returned as the associated entity.

        :param original_entity:
        :return: associated entity
        """
        # Get all entities
        entities = self.robot.ed.get_entities()

        # Remove all entities with a shape. These are probably not the ones we want to grasp
        for e in entities:
            if e.is_a("furniture"):
                entities.remove(e)
        entities = sorted(entities,
                          key=lambda entity: entity.distance_to_3d(original_entity._pose.p))

        if self.distance(entities[0], original_entity) < 0.05:  # Objects Less than 5 cm apart might be associated
            return entities[0]
        else:
            return original_entity

    @staticmethod
    def distance(e1, e2):
        """ Computes the distance between two entities """
        return e1.distance_to_3d(e2._pose.p)


class ResetOnFailure(smach.State):
    """ Class to reset the robot after a grab has failed """

    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING], }

    def __init__(self, robot, arm):
        """
        Constructor

        :param robot: robot object
        :param arm: arm designator
        """
        smach.State.__init__(self, outcomes=['done'])

        self._robot = robot
        self.arm_designator = arm

    def execute(self, userdata=None):
        """ Execute hook """
        arm = self.arm_designator.resolve()
        arm.reset()

        if self._robot.robot_name == "amigo":
            self._robot.torso.reset()  # Move up to make resetting of the arm safer.
        if arm is not None:
            arm.gripper.send_goal('close')
        self._robot.head.reset()  # Sends a goal
        self._robot.head.cancel_goal()  # And cancels it...
        if arm is not None:
            arm.reset()
        self._robot.torso.reset()
        return 'done'


class Grab(smach.StateMachine):
    def __init__(self, robot: Robot, item: Designator, arm: ArmDesignator, room: Designator = None):
        """
        Let the given robot move to an entity and grab that entity using some arm

        :param robot: Robot to use
        :param item: Designator that resolves to the item to grab. E.g. EntityByIdDesignator
        :param arm: Designator that resolves to the arm to use for grabbing. E.g. UnoccupiedArmDesignator
        :param room: Designator that resolves to the room where the robot has to stay in. E.g. EntityByIdDesignator
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types
        check_type(item, Entity)
        check_type(arm, PublicArm)
        check_type(room, Entity, type(None))

        with self:
            smach.StateMachine.add('RESOLVE_ARM', ResolveArm(arm, self),
                                   transitions={'succeeded': 'NAVIGATE_TO_GRAB',
                                                'failed': 'failed'})

            smach.StateMachine.add('NAVIGATE_TO_GRAB', NavigateToGrasp(robot, arm, item, room),
                                   transitions={'unreachable': 'RESET_FAILURE',
                                                'goal_not_defined': 'RESET_FAILURE',
                                                'arrived': 'PREPARE_GRASP'})

            smach.StateMachine.add('PREPARE_GRASP', PrepareEdGrasp(robot, arm, item),
                                   transitions={'succeeded': 'GRAB',
                                                'failed': 'RESET_FAILURE'})

            smach.StateMachine.add('GRAB', PickUp(robot, arm, item),
                                   transitions={'succeeded': 'done',
                                                'failed': 'RESET_FAILURE'})

            smach.StateMachine.add("RESET_FAILURE", ResetOnFailure(robot, arm),
                                   transitions={'done': 'failed'})

        check_arm_requirements(self, robot)
