#! /usr/bin/env python
import math
import rospy
import smach
import tf
from robot_skills.util.kdl_conversions import kdlFrameFromXYZRPY

from robot_skills.util.entity import Entity
import robot_skills.util.msg_constructors as msgs
import robot_skills.util.transformations as transformations

from robot_skills.arms import Arm
from robot_smach_states.util.designators import check_type

from robot_smach_states.navigation import NavigateToGrasp
from robot_smach_states.manipulation.grasp_point_determination import GraspPointDeterminant


class PrepareEdGrasp(smach.State):
    def __init__(self, robot, arm, grab_entity):
        """
        Set the arm in the appropriate position before actually grabbing
        :param robot: robot to execute state with
        :param arm: Designator that resolves to arm to grab with. E.g. UnoccupiedArmDesignator
        :param grab_entity: Designator that resolves to the entity to grab. e.g EntityByIdDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], output_keys=['arm'])

        # Assign member variables
        self.robot = robot
        self.arm_designator = arm
        self.grab_entity_designator = grab_entity

        check_type(grab_entity, Entity)

    def execute(self, userdata):

        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"
        userdata.arm = arm.side

        entity = self.grab_entity_designator.resolve()
        if not entity:
            rospy.logerr("Could not resolve grab_entity")
            return "failed"

        # Open gripper (non-blocking)
        arm.send_gripper_goal('open', timeout=0)

        # Torso up (non-blocking)
        self.robot.torso.high()

        # Arm to position in a safe way
        arm.send_joint_trajectory('prepare_grasp', timeout=0)

        # Open gripper
        arm.send_gripper_goal('open', timeout=0.0)

        # Make sure the head looks at the entity
        pos = entity.pose.position
        self.robot.head.look_at_point(msgs.PointStamped(pos.x, pos.y, pos.z, "/map"), timeout=0.0)

        return 'succeeded'


class PickUp(smach.State):
    def __init__(self, robot, arm, grab_entity):
        """
        Pick up an item given an arm and an entity to be picked up
        :param robot: robot to execute this state with
        :param arm: Designator that resolves to the arm to grab the grab_entity with. E.g. UnoccupiedArmDesignator
        :param grab_entity: Designator that resolves to the entity to grab. e.g EntityByIdDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], output_keys=['arm'])

        # Assign member variables
        self.robot = robot
        self.arm_designator = arm

        check_type(grab_entity, Entity)
        self.grab_entity_designator = grab_entity
        self._gpd = GraspPointDeterminant(robot)

    def execute(self, userdata):

        grab_entity = self.grab_entity_designator.resolve()
        if not grab_entity:
            rospy.logerr("Could not resolve grab_entity")
            return "failed"

        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"
        userdata.arm = arm.side  # Using userdata makes sure we don't need to do any more arm entity magic

        # goal in map frame
        goal_map = msgs.Point(0, 0, 0)

        try:
            # Transform to base link frame
            goal_bl = transformations.tf_transform(
                goal_map, grab_entity.id, self.robot.robot_name+'/base_link',
                tf_listener=self.robot.tf_listener
            )
            if goal_bl is None:
                rospy.logerr('Transformation of goal to base failed')
                return 'failed'
        except tf.Exception, tfe:
            rospy.logerr('Transformation of goal to base failed: {0}'.format(tfe))
            return 'failed'

        # Make sure the torso and the arm are done
        self.robot.torso.wait_for_motion_done()
        arm.wait_for_motion_done()

        # This is needed because the head is not entirely still when the
        # look_at_point function finishes
        rospy.sleep(rospy.Duration(0.5))

        # Update the entity (position)
        segm_res = self.robot.ed.update_kinect("%s" % grab_entity.id)

        # Resolve the entity again because we want the latest pose
        updated_grab_entity = self.grab_entity_designator.resolve()

        rospy.loginfo("ID to update: {0}".format(grab_entity.id))
        if not updated_grab_entity:
            rospy.logerr("Could not resolve the updated grab_entity, "
                         "this should not happen [CHECK WHY THIS IS HAPPENING]")
            grab_entity = self.associate(original_entity=grab_entity)
        else:
            rospy.loginfo("Updated pose of entity (dx, dy, dz) : (%f, %f, %f)" %
                          (updated_grab_entity.pose.position.x - grab_entity.pose.position.x,
                           updated_grab_entity.pose.position.y - grab_entity.pose.position.y,
                           updated_grab_entity.pose.position.z - grab_entity.pose.position.z))
            grab_entity = updated_grab_entity

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Grasp point determination
        grasp_pose = self._gpd.get_grasp_pose(grab_entity, arm)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        # goal in map frame
        goal_map = msgs.Point(0, 0, 0)

        #try:
        # In case grasp point determination didn't work
        if not grasp_pose:
            if self.robot.tf_listener.waitForTransform(grab_entity.id, self.robot.robot_name + "/base_link"):
                # Transform to base link frame
                goal_bl = transformations.tf_transform(goal_map, grab_entity.id, self.robot.robot_name + "/base_link",
                                                       tf_listener=self.robot.tf_listener)
                if goal_bl is None:
                    return 'failed'
            else:
                return 'failed'
        else:
            # We do have a grasp pose, given as a kdl frame in map
            if self.robot.tf_listener.waitForTransform("/map", self.robot.robot_name + "/base_link"):
                # Transform to base link frame
                goal_bl = transformations.tf_transform(msgs.Point(grasp_pose.p.x(),
                                                                  grasp_pose.p.y(),
                                                                  grasp_pose.p.z()),
                                                       "/map", self.robot.robot_name + "/base_link",
                                                       tf_listener=self.robot.tf_listener)
                if goal_bl is None:
                    return 'failed'
            else:
                return 'failed'

        #except tf.Exception as tfe:
        #    rospy.logerr('Transformation of goal to base failed: {0}'.format(tfe))
        #    return 'failed'

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
        # rospy.loginfo('Start grasping')
        if not arm.send_goal(kdlFrameFromXYZRPY(goal_bl.x, goal_bl.y, goal_bl.z, 0, 0, 0),
                             timeout=20, pre_grasp=True,
                             allowed_touch_objects=[grab_entity.id]
                             ):
            self.robot.speech.speak('I am sorry but I cannot move my arm to the object position', block=False)
            rospy.logerr('Grasp failed')
            arm.reset()
            arm.send_gripper_goal('close', timeout=0.0)
            return 'failed'

        # Close gripper
        arm.send_gripper_goal('close')

        arm.occupied_by = grab_entity

        # Lift
        # rospy.loginfo('Start lifting')
        if arm.side == "left":
            roll = 0.3
        else:
            roll = -0.3
        if not arm.send_goal(kdlFrameFromXYZRPY(goal_bl.x, goal_bl.y, goal_bl.z + 0.05, roll, 0.0, 0.0),
                             timeout=20, allowed_touch_objects=[grab_entity.id]
                             ):
            rospy.logerr('Failed lift')

        # Retract
        # rospy.loginfo('Start retracting')
        if arm.side == "left":
            roll = 0.6
        else:
            roll = -0.6
        if not arm.send_goal(kdlFrameFromXYZRPY(goal_bl.x - 0.1, goal_bl.y, goal_bl.z + 0.05, roll, 0.0, 0.0),
                             timeout=0.0, allowed_touch_objects=[grab_entity.id]
                             ):
            rospy.logerr('Failed retract')

        self.robot.base.force_drive(-0.125, 0, 0, 2.0)

        # Update Kinect once again to make sure the object disappears from ED
        segm_res = self.robot.ed.update_kinect("%s" % grab_entity.id)

        arm.wait_for_motion_done()

        # Carrying pose
        # rospy.loginfo('start moving to carrying pose')
        arm.send_joint_goal('carrying_pose', timeout=0.0)

        # Reset head
        self.robot.head.cancel_goal()

        return 'succeeded'

    def associate(self, original_entity):
        """ Tries to associate the original entity with one of the entities in the world model. This is useful if
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
                          key=lambda entity: entity.distance_to_3d(original_entity.pose.p))

        if self.distance(entities[0], original_entity) < 0.05:  # Objects Less than 5 cm apart might be associated
            return entities[0]
        else:
            return original_entity


class ResetOnFailure(smach.StateMachine):
    """ Class to reset the robot after a grab has failed """
    def __init__(self, robot):
        """ Constructor
        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=['done'], input_keys=['arm'])

        self._robot = robot

    def execute(self, userdata):
        """ Execute hook """
        if hasattr(userdata, 'arm'):
            arm = self._robot.get_arm(userdata.arm)[0]  # Using userdata makes sure we don't need more arm designator magic
        else:
            arm = None
        self._robot.torso.high()  # Move up to make resetting of the arm safer
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
        :return:
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types
        check_type(item, Entity)
        check_type(arm, Arm)

        with self:
            smach.StateMachine.add('PREPARE_GRASP', PrepareEdGrasp(robot, arm, item),
                                   transitions={'succeeded': 'NAVIGATE_TO_GRAB',
                                                'failed': 'RESET_FAILURE'})

            smach.StateMachine.add('NAVIGATE_TO_GRAB', NavigateToGrasp(robot, item, arm),
                                   transitions={'unreachable': 'RESET_FAILURE',
                                                'goal_not_defined': 'RESET_FAILURE',
                                                'arrived': 'GRAB'})

            smach.StateMachine.add('GRAB', PickUp(robot, arm, item),
                                   transitions={'succeeded': 'done',
                                                'failed': 'RESET_FAILURE'})

            smach.StateMachine.add("RESET_FAILURE", ResetOnFailure(robot),
                                   transitions={'done': 'failed'})
