from __future__ import absolute_import

# System
import math

# ROS
import PyKDL as kdl
import rospy
import smach
import tf2_ros

# TU/e Robotics
from robot_skills.util.kdl_conversions import VectorStamped, FrameStamped
from robot_skills.util.entity import Entity
from robot_skills.arms import PublicArm
from robot_skills.robot import Robot
from ..util.designators import check_type
from ..navigation.navigate_to_grasp import NavigateToGrasp
from ..util.designators.arm import ArmDesignator
from ..util.designators.core import Designator


def get_grasp_pose(robot, entity, arm):
    center_pose = entity._pose

    ''' Get robot pose as a kdl frame (is required later on) '''
    robot_frame = robot.base.get_location()
    robot_frame_inv = robot_frame.frame.Inverse()

    # Determine yaw
    yaw = math.atan2(center_pose.p.y() - robot_frame.frame.p.y(), center_pose.p.x() - robot_frame.frame.p.x())
    rospy.loginfo("yaw: {}".format(yaw))

    grasp_pose = FrameStamped(kdl.Frame(kdl.Rotation.RPY(0, 0, yaw),
                                        center_pose.p),
                              frame_id="/map")
    return grasp_pose


class PickUpCard(smach.State):
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

        # confirm location of object
        self.robot.move_to_inspect_pose(grab_entity._pose.p)
        self.robot.head.look_at_point(VectorStamped(vector=grab_entity._pose.p, frame_id="/map"), timeout=0.0)
        self.robot.head.wait_for_motion_done()
        segm_res = self.robot.ed.update_kinect("%s" % grab_entity.id)

        # Arm to position in a safe way
        self.robot.move_to_pregrasp_pose(arm, grab_entity._pose.p)
        arm.send_gripper_goal('close', timeout=0.0)
        arm.wait_for_motion_done()

        # Make sure the head looks at the entity (so you can see where its going to grasp)
        self.robot.head.look_at_point(VectorStamped(vector=grab_entity._pose.p, frame_id="/map"), timeout=0.0)

        # Make sure the torso and the arm are done
        self.robot.torso.wait_for_motion_done(cancel=True)
        arm.wait_for_motion_done(cancel=True)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Grasp point determination
        grasp_framestamped = get_grasp_pose(self.robot, grab_entity, arm)

        # TODO magic numbers
        # define transform from handpalm to suction cup
        M = kdl.Rotation.Quaternion(-0.6644307973614778,
                                     -0.24193328636386194,
                                     -0.6644307970373836,
                                     0.24193328918327617) \
             * kdl.Rotation(0, 1, 0, 1, 0, 0, 0, 0, -1) # rotation frame because toyota uses a different frame convention
        fingervacuum_transform = kdl.Frame(M,
                                           kdl.Vector(0.0864371733535394, 0.028381482562915693, -0.012000000053262072))
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        grasp_framestamped.frame = grasp_framestamped.frame * fingervacuum_transform.Inverse()

        rospy.loginfo("M : {}\n grasp_framestamped: {}".format(M, grasp_framestamped))

        # In case grasp point determination didn't work
        if not grasp_framestamped:
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

        # Grasp
        goal_bl.frame.p.z(goal_bl.frame.p.z() + 0.05)  # Add 5 cm

        rospy.loginfo('Start grasping')
        if not arm.send_goal(goal_bl, timeout=20, pre_grasp=True, allowed_touch_objects=[grab_entity.id]):
            self.robot.speech.speak('I am sorry but I cannot move my arm to the object position', block=False)
            rospy.logerr('Grasp failed to {}'.format(goal_bl))
            arm.reset()
            return 'failed'

        # Picking
        goal_bl.frame.p.z(goal_bl.frame.p.z() - 0.05)  # Remove 5 cm

        rospy.loginfo('Start lowering')
        if not arm.send_goal(goal_bl, timeout=20, pre_grasp=True, allowed_touch_objects=[grab_entity.id]):
            self.robot.speech.speak('I am sorry but I cannot move my arm to the object position', block=False)
            rospy.logerr('Grasp failed to {}'.format(goal_bl))
            arm.reset()
            return 'failed'
        # pick
        # TODO use suction cup

        arm.occupied_by = grab_entity

        # Lift
        goal_bl = grasp_framestamped.projectToFrame(self.robot.robot_name + "/base_link",
                                                    tf_listener=self.robot.tf_listener)
        rospy.loginfo('Start lifting')

        goal_bl.frame.p.z(goal_bl.frame.p.z() + 0.05)  # Add 5 cm
        rospy.loginfo("Start lift")
        if not arm.send_goal(goal_bl, timeout=20, allowed_touch_objects=[grab_entity.id]):
            rospy.logerr('Failed lift')

        # Retract
        goal_bl = grasp_framestamped.projectToFrame(self.robot.robot_name + '/base_link',
                                                    tf_listener=self.robot.tf_listener)
        rospy.loginfo('Start retracting')

        goal_bl.frame.p.x(goal_bl.frame.p.x() - 0.1)  # Retract 10 cm
        goal_bl.frame.p.z(goal_bl.frame.p.z() + 0.05)  # Go 5 cm higher
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


class GrabCard(smach.StateMachine):
    def __init__(self, robot, item, arm):
        """
        Let the given robot move to an entity and grab that entity using some arm with a suction cup

        :param robot: Robot to use
        :param item: Designator that resolves to the item to grab. E.g. EntityByIdDesignator
        :param arm: Designator that resolves to the arm to use for grabbing. Eg. UnoccupiedArmDesignator
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types
        check_type(item, Entity)
        check_type(arm, PublicArm)

        with self:
            smach.StateMachine.add('NAVIGATE_TO_GRAB', NavigateToGrasp(robot, item, arm),
                                   transitions={'unreachable': 'RESET_FAILURE',
                                                'goal_not_defined': 'RESET_FAILURE',
                                                'arrived': 'PICKUP'})

            smach.StateMachine.add('PICKUP', PickUpCard(robot, arm, item),
                                   transitions={'succeeded': 'done',
                                                'failed': 'RESET_FAILURE'})

            smach.StateMachine.add("RESET_FAILURE", ResetOnFailure(robot, arm),
                                   transitions={'done': 'failed'})
