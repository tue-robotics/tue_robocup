# ROS
import PyKDL as kdl
from pykdl_ros import FrameStamped, VectorStamped
import rospy
import smach
import tf2_ros

# TU/e Robotics
from ed.entity import Entity

import robot_smach_states.util.designators as ds
from robot_smach_states.designator_iterator import IterateDesignator
from robot_smach_states.human_interaction import AskYesNo, Say
from robot_smach_states.manipulation import PrepareEdGrasp, ResetOnFailure
from robot_smach_states.navigation import NavigateToGrasp
from robot_smach_states.utility import WaitTime
from robot_skills.arm.arms import PublicArm
from robot_smach_states.utility import ResolveArm, check_arm_requirements


class PointAt(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_goals": ["carrying_pose"], }

    def __init__(self, robot, arm, point_entity_designator):
        """
        Points at an item, similar to picking it up.

        :param robot: robot to execute this state with
        :param arm: Designator that resolves to the arm to point at the entity with. E.g. UnoccupiedArmDesignator
        :param point_entity_designator: Designator that resolves to the entity to point at. e.g EntityByIdDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Assign member variables
        self.robot = robot
        self.arm_des = arm

        ds.check_type(point_entity_designator, Entity)
        self.point_entity_designator = point_entity_designator

    def execute(self, userdata=None):

        point_entity = self.point_entity_designator.resolve()
        if not point_entity:
            rospy.logerr("Could not resolve grab_entity")
            return "failed"

        arm = self.arm_des.resolve()

        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        goal_map = VectorStamped.from_xyz(0, 0, 0, rospy.Time(), point_entity.uuid)

        try:
            # Transform to base link frame
            goal_bl = self.robot.tf_buffer.transform(goal_map, self.robot.base_link_frame)
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
        rospy.sleep(0.5)

        # Resolve the entity again because we want the latest pose
        updated_point_entity = self.point_entity_designator.resolve()  # type: Entity

        rospy.loginfo("ID to update: {0}".format(point_entity.uuid))
        if not updated_point_entity:
            rospy.logerr("Could not resolve the updated grab_entity, "
                         "this should not happen [CHECK WHY THIS IS HAPPENING]")
            point_entity = self.associate(original_entity=point_entity)
        else:
            rospy.loginfo("Updated pose of entity (dx, dy, dz) : (%f, %f, %f)" %
                          (updated_point_entity.pose.frame.p.x() - point_entity.pose.frame.p.x(),
                           updated_point_entity.pose.frame.p.y() - point_entity.pose.frame.p.y(),
                           updated_point_entity.pose.frame.p.z() - point_entity.pose.frame.p.z()))
            point_entity = updated_point_entity
            point_entity._last_update_time = rospy.Time()  # Just the most recent time ToDo: Hack

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Grasp point determination
        goal_bl = self._get_pointing_pose(point_entity)

        # Pointing with gripper closed is better
        arm.gripper.send_goal('close')

        # Grasp
        rospy.loginfo('Start pointing')
        one_pointing_succeeded = False
        z_offset = 0.02  # 2cm
        for x_offset in [-0.15, 0.0]:  # Hack because Hero does not pre-grasp reliably
            _goal_bl = FrameStamped(goal_bl.frame * kdl.Frame(kdl.Vector(x_offset, 0.0, z_offset)), rospy.Time(),
                                    goal_bl.header.frame_id)
            if not arm.send_goal(_goal_bl, timeout=20, pre_grasp=False, allowed_touch_objects=[point_entity.uuid]):
                self.robot.speech.speak('I am sorry but I cannot move my arm to the object position', block=False)
                rospy.logerr('Pointing failed')
                arm.reset()
            else:
                one_pointing_succeeded = True

        if not one_pointing_succeeded:
            rospy.logerr("Both Pointing at failed")
            return 'failed'

        # Retract
        rospy.loginfo('Start retracting')
        roll = 0.0

        goal_bl.frame.p.x(goal_bl.frame.p.x() - 0.1)  # Retract 10 cm
        goal_bl.frame.p.z(goal_bl.frame.p.z() + 0.05)  # Go 5 cm higher
        goal_bl.frame.M = kdl.Rotation.RPY(roll, 0.0, 0.0)  # Update the roll
        rospy.loginfo("Start retract")
        if not arm.send_goal(goal_bl, timeout=0.0, allowed_touch_objects=[point_entity.uuid]):
            rospy.logerr('Failed retract')
        arm.wait_for_motion_done()
        self.robot.base.force_drive(-0.125, 0, 0, 2.0)

        # Close gripper
        arm.wait_for_motion_done(cancel=True)

        # Carrying pose
        arm.send_joint_goal('carrying_pose', timeout=0.0)

        # Reset head
        self.robot.head.cancel_goal()

        return "succeeded"

    def _get_pointing_pose(self, point_entity):
        # type: (Entity, Arm) -> FrameStamped
        """
        Computes the pointing pose, i.e., where the grippoint should go to. The position is equal to the position of
        the entity. The orientation is determined such that it easily aligns with the robot.

        :param point_entity: (Entity)
        :return: (FrameStamped)
        """
        # Compute the frame w.r.t. base link
        fs_robot = self.robot.tf_buffer.transform(point_entity.pose, self.robot.base_link_frame)

        # Set the orientation to unity
        fs_robot.frame.M = kdl.Rotation()

        return fs_robot


class IdentifyObject(smach.StateMachine):
    def __init__(self, robot, items, arm):
        """
        Has the robot points to the object that was pointed to by the operator.

        This is based on the 'Grab' state, with difference that pickup is changed, i.e., without closing gripper.

        :param robot: Robot to use
        :param items: Designator that resolves to the list of items to point.
        :param arm: Designator that resolves to the arm to use for grabbing. Eg. UnoccupiedArmDesignator
        :return:
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types
        ds.check_type(items, [Entity])
        ds.check_type(arm, PublicArm)

        item = ds.VariableDesignator(resolve_type=Entity)

        with self:
            smach.StateMachine.add('RESOLVE_ARM', ResolveArm(arm, self),
                                   transitions={'succeeded': 'ITERATE_ITEM',
                                                'failed': 'failed'})

            smach.StateMachine.add("ITERATE_ITEM", IterateDesignator(items, item.writeable),
                                   transitions={'next': 'NAVIGATE_TO_POINT',
                                                'stop_iteration': 'failed'})

            smach.StateMachine.add('NAVIGATE_TO_POINT', NavigateToGrasp(robot, arm, item),
                                   transitions={'unreachable': 'WAIT_BEFORE_NAVIGATE_BACKUP',
                                                'goal_not_defined': 'WAIT_BEFORE_NAVIGATE_BACKUP',
                                                'arrived': 'PREPARE_POINT'})

            smach.StateMachine.add("ASK_OPERATOR_MOVE", Say(robot, "Operator, please move, so I can drive"),
                                   transitions={'spoken': "WAIT_BEFORE_NAVIGATE_BACKUP"})

            smach.StateMachine.add('WAIT_BEFORE_NAVIGATE_BACKUP', WaitTime(robot, 5),
                                   transitions={'waited': 'NAVIGATE_TO_POINT_BACKUP',
                                                'preempted': 'failed'})

            smach.StateMachine.add('NAVIGATE_TO_POINT_BACKUP', NavigateToGrasp(robot, arm, item),
                                   transitions={'unreachable': 'RESET_FAILURE',
                                                'goal_not_defined': 'RESET_FAILURE',
                                                'arrived': 'PREPARE_POINT'})

            smach.StateMachine.add('PREPARE_POINT', PrepareEdGrasp(robot, arm, item),
                                   transitions={'succeeded': 'POINT',
                                                'failed': 'RESET_FAILURE'})

            smach.StateMachine.add('POINT', PointAt(robot, arm, item),
                                   transitions={'succeeded': 'ASK_CORRECT',
                                                'failed': 'RESET_FAILURE'})

            smach.StateMachine.add("ASK_CORRECT", Say(robot, ["Is this the object you pointed at?",
                                                              "Did I got the correct object?"
                                                              ],
                                                      block=True,
                                                      look_at_standing_person=True),
                                   transitions={'spoken': 'LISTEN_ANSWER'})

            smach.StateMachine.add("LISTEN_ANSWER", AskYesNo(robot),
                                   transitions={'yes': 'done',
                                                'no': 'SAY_ANOTHER_GUESS',
                                                'no_result': 'done'})

            smach.StateMachine.add("SAY_ANOTHER_GUESS", Say(robot, ["Let me maybe try another guess"]),
                                   transitions={'spoken': 'ITERATE_ITEM'})

            smach.StateMachine.add("RESET_FAILURE", ResetOnFailure(robot, arm),
                                   transitions={'done': 'failed'})

            check_arm_requirements(self, robot)


if __name__ == "__main__":
    from robot_skills import get_robot_from_argv

    rospy.init_node("test_grasping")

    robot = get_robot_from_argv(index=1)

    entity_id = "test_item"
    pose = FrameStamped(kdl.Frame(kdl.Rotation.RPY(0, 0, -1.57), kdl.Vector(2.6, -0.95, 0.8)), rospy.Time.now(), "map")

    robot.ed.update_entity(uuid=entity_id, frame_stamped=pose)

    item = ds.EdEntityDesignator(robot, uuid=entity_id)

    arm = ds.UnoccupiedArmDesignator(robot, {})

    grab_state = IdentifyObject(robot, item, arm)
    grab_state.execute()
