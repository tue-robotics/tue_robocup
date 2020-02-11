# ROS
import PyKDL as kdl
import rospy
import smach
import tf2_ros

# TU/e Robotics
from robot_skills.util.kdl_conversions import VectorStamped
from robot_skills.util.entity import Entity
from robot_skills.arms import PublicArm, GripperMeasurement
from robot_skills.robot import Robot
from robot_smach_states.util.designators import check_type
from robot_smach_states.manipulation.grasp_point_determination import GraspPointDeterminant
from robot_smach_states.util.designators.arm import ArmDesignator
from robot_smach_states.util.designators.core import Designator

from .move_to_grasp import MoveToGrasp


class PrepareEdGrasp(smach.State):
    def __init__(self, robot, arm, grab_entity):
        # type: (Robot, ArmDesignator, Designator) -> None
        """
        Set the arm in the appropriate position before actually grabbing

        :param robot: robot to execute state with
        :param arm: Designator that resolves to arm to grab with. E.g. UnoccupiedArmDesignator
        :param grab_entity: Designator that resolves to the entity to grab. e.g EntityByIdDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Assign member variables
        self.robot = robot
        self.arm_designator = arm
        self.grab_entity_designator = grab_entity

        check_type(grab_entity, Entity)

    def execute(self, userdata=None):
        entity = self.grab_entity_designator.resolve()
        if not entity:
            rospy.logerr("Could not resolve grab_entity")
            return "failed"

        self.robot.head.look_at_point(VectorStamped(vector=entity._pose.p, frame_id="/map"), timeout=0.0)
        self.robot.head.wait_for_motion_done()
        segm_res = self.robot.ed.update_kinect("%s" % entity.id)

        arm = self.arm_designator.resolve()

        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        # Torso up (non-blocking)
        self.robot.torso.reset()

        # Arm to position in a safe way
        arm.send_joint_trajectory('prepare_grasp', timeout=0)
        arm.wait_for_motion_done()

        # Open gripper
        arm.send_gripper_goal('open', timeout=0.0)
        arm.wait_for_motion_done()

        # Make sure the head looks at the entity
        self.robot.head.look_at_point(VectorStamped(vector=entity._pose.p, frame_id="/map"), timeout=0.0)
        self.robot.head.wait_for_motion_done()
        return 'succeeded'


class PickUp(smach.State):
    def __init__(self, robot, arm, grab_entity, check_occupancy=False):
        """
        Pick up an item given an arm and an entity to be picked up

        :param robot: robot to execute this state with
        :param arm: Designator that resolves to the arm to grab the grab_entity with. E.g. UnoccupiedArmDesignator
        :param grab_entity: Designator that resolves to the entity to grab. e.g EntityByIdDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Assign member variables
        self.robot = robot
        self.arm_designator = arm

        check_type(grab_entity, Entity)
        self.grab_entity_designator = grab_entity
        self._gpd = GraspPointDeterminant(robot)
        self._check_occupancy = check_occupancy

    def execute(self, userdata=None):

        grab_entity = self.grab_entity_designator.resolve()
        if not grab_entity:
            rospy.logerr("Could not resolve grab_entity")
            return "failed"

        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        goal_map = VectorStamped(0, 0, 0, frame_id=grab_entity.id)

        try:
            # Transform to base link frame
            goal_bl = goal_map.projectToFrame(self.robot.robot_name+'/base_link', tf_listener=self.robot.tf_listener)
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

        rospy.loginfo("ID to update: {0}".format(grab_entity.id))
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

        goal_map = VectorStamped(0, 0, 0, frame_id=grab_entity.id)

        # In case grasp point determination didn't work
        if not grasp_framestamped:
            goal_bl = goal_map.projectToFrame(self.robot.robot_name + '/base_link', tf_listener=self.robot.tf_listener)
            if goal_bl is None:
                return 'failed'
            else:
                return 'failed'
        else:
            # We do have a grasp pose, given as a kdl frame in map
            try:
                self.robot.tf_listener.waitForTransform("/map", self.robot.robot_name + "/base_link", rospy.Time(0),
                                                        rospy.Duration(10))
                # Transform to base link frame
                goal_bl = grasp_framestamped.projectToFrame(self.robot.robot_name + "/base_link",
                                                            tf_listener=self.robot.tf_listener)
                if goal_bl is None:
                    return 'failed'
            except tf2_ros.TransformException as tfe:
                rospy.logerr('Transformation of goal to base failed: {0}'.format(tfe))
                return 'failed'

        # Pre-grasp --> this is only necessary when using visual servoing
        # rospy.loginfo('Starting Pre-grasp')
        # if not arm.send_goal(goal_bl.x, goal_bl.y, goal_bl.z, 0, 0, 0,
        #                      frame_id='/'+self.robot.robot_name+'/base_link',
        #                      timeout=20, pre_grasp=True, first_joint_pos_only=True
        #                      ):
        #     rospy.logerr('Pre-grasp failed:')
        #     arm.reset()
        #     arm.send_gripper_goal('close', timeout=None)
        #     return 'failed'

        # Grasp
        rospy.loginfo('Start grasping')
        if not arm.send_goal(goal_bl, timeout=20, pre_grasp=True, allowed_touch_objects=[grab_entity.id]):
            self.robot.speech.speak('I am sorry but I cannot move my arm to the object position', block=False)
            rospy.logerr('Grasp failed')
            arm.reset()
            arm.send_gripper_goal('close', timeout=0.0)
            return 'failed'

        # Close gripper
        arm.send_gripper_goal('close')

        arm.occupied_by = grab_entity

        # Lift
        goal_bl = grasp_framestamped.projectToFrame(self.robot.robot_name + "/base_link",
                                                    tf_listener=self.robot.tf_listener)
        rospy.loginfo('Start lifting')
        roll = 0.0

        goal_bl.frame.p.z(goal_bl.frame.p.z() + 0.05)  # Add 5 cm
        goal_bl.frame.M = kdl.Rotation.RPY(roll, 0, 0)  # Update the roll
        rospy.loginfo("Start lift")
        if not arm.send_goal(goal_bl, timeout=20, allowed_touch_objects=[grab_entity.id]):
            rospy.logerr('Failed lift')

        # Retract
        goal_bl = grasp_framestamped.projectToFrame(self.robot.robot_name + '/base_link',
                                                    tf_listener=self.robot.tf_listener)
        rospy.loginfo('Start retracting')
        roll = 0.0

        goal_bl.frame.p.x(goal_bl.frame.p.x() - 0.1)  # Retract 10 cm
        goal_bl.frame.p.z(goal_bl.frame.p.z() + 0.05)  # Go 5 cm higher
        goal_bl.frame.M = kdl.Rotation.RPY(roll, 0.0, 0.0)  # Update the roll
        rospy.loginfo("Start retract")
        if not arm.send_goal(goal_bl, timeout=0.0, allowed_touch_objects=[grab_entity.id]):
            rospy.logerr('Failed retract')
        arm.wait_for_motion_done()
        self.robot.base.force_drive(-0.125, 0, 0, 2.0)

        # Update Kinect once again to make sure the object disappears from ED
        segm_res = self.robot.ed.update_kinect("%s" % grab_entity.id)

        arm.wait_for_motion_done(cancel=True)

        # Carrying pose
        # rospy.loginfo('start moving to carrying pose')
        arm.send_joint_goal('carrying_pose', timeout=0.0)

        result = 'succeeded'
        if self._check_occupancy:
            # Check if the object is present in the gripper
            if arm.object_in_gripper_measurement.is_empty:
                # If state is empty, grasp has failed
                result = "failed"
                rospy.logerr("Gripper is not holding an object")
                self.robot.speech.speak("Whoops, something went terribly wrong")
                arm.occupied_by = None  # Set the object the arm is holding to None
            else:
                # State is holding, grasp succeeded.
                # If unknown: sensor not there, assume gripper is holding and hope for the best
                result = "succeeded"
                if arm.object_in_gripper_measurement.is_unknown:
                    rospy.logwarn("GripperMeasurement unknown")

        # Reset head
        self.robot.head.cancel_goal()

        return result

    def associate(self, original_entity):
        """
        Tries to associate the original entity with one of the entities in the world model. This is useful if
        after an update, the original entity is no longer present in the world model. If no good map can be found,
        the original entity will be returned as the associated entity.

        :param original_entity:
        :return: associated entity
        """
        # Get all entities
        entities = self.robot.ed.get_entities(parse=False)

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


class ResetOnFailure(smach.StateMachine):
    """ Class to reset the robot after a grab has failed """
    def __init__(self, robot, arm):
        """
        Constructor

        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=['done'])

        self._robot = robot
        self.arm_designator = arm

    def execute(self, userdata=None):
        """ Execute hook """
        arm = self.arm_designator.resolve()
        arm.reset()

        if self._robot.robot_name == "amigo":
            self._robot.torso.reset()  # Move up to make resetting of the arm safer.
        if arm is not None:
            arm.send_gripper_goal('close')
        self._robot.head.reset()  # Sends a goal
        self._robot.head.cancel_goal()  # And cancels it...
        if arm is not None:
            arm.reset()
        self._robot.torso.reset()
        return 'done'


class Grab(smach.StateMachine):
    def __init__(self, robot, item, arm):
        """
        Let the given robot move to an entity and grab that entity using some arm

        :param robot: Robot to use
        :param item: Designator that resolves to the item to grab. E.g. EntityByIdDesignator
        :param arm: Designator that resolves to the arm to use for grabbing. Eg. UnoccupiedArmDesignator
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types
        check_type(item, Entity)
        check_type(arm, PublicArm)

        with self:
            smach.StateMachine.add('MOVE_TO_GRAB', MoveToGrasp(robot, item, arm),
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
