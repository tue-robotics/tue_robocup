# ROS
import PyKDL as kdl
import rospy
import smach
import tf2_ros

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills import get_robot_from_argv
from robot_skills.arms import PublicArm
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import VectorStamped, FrameStamped


class PointAt(smach.State):
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
        self.arm_designator = arm

        states.check_type(point_entity_designator, Entity)
        self.point_entity_designator = point_entity_designator

    def execute(self, userdata=None):

        point_entity = self.point_entity_designator.resolve()
        if not point_entity:
            rospy.logerr("Could not resolve grab_entity")
            return "failed"

        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        goal_map = VectorStamped(0, 0, 0, frame_id=point_entity.id)

        try:
            # Transform to base link frame
            goal_bl = goal_map.projectToFrame(self.robot.base_link_frame, tf_listener=self.robot.tf_listener)
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
        updated_point_entity = self.point_entity_designator.resolve()

        rospy.loginfo("ID to update: {0}".format(point_entity.id))
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

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Grasp point determination
        goal_bl = self._get_pointing_pose(point_entity)

        # Grasp
        rospy.loginfo('Start pointing')
        for x_offset in [-0.15, 0.0]:  # Hack because Hero does not pre-grasp reliably
            _goal_bl = FrameStamped(goal_bl.frame * kdl.Frame(kdl.Vector(x_offset, 0.0, 0.0)), goal_bl.frame_id)
            if not arm.send_goal(_goal_bl, timeout=20, pre_grasp=False, allowed_touch_objects=[point_entity.id]):
                self.robot.speech.speak('I am sorry but I cannot move my arm to the object position', block=False)
                rospy.logerr('Grasp failed')
                arm.reset()
                return 'failed'

        # Retract
        rospy.loginfo('Start retracting')
        roll = 0.0

        goal_bl.frame.p.x(goal_bl.frame.p.x() - 0.1)  # Retract 10 cm
        goal_bl.frame.p.z(goal_bl.frame.p.z() + 0.05)  # Go 5 cm higher
        goal_bl.frame.M = kdl.Rotation.RPY(roll, 0.0, 0.0)  # Update the roll
        rospy.loginfo("Start retract")
        if not arm.send_goal(goal_bl, timeout=0.0, allowed_touch_objects=[point_entity.id]):
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
        fs_robot = point_entity.pose.projectToFrame(self.robot.base_link_frame, self.robot.tf_listener)

        # Set the orientation to unity
        fs_robot.frame.M = kdl.Rotation()

        return fs_robot


class IdentifyObject(smach.StateMachine):
    def __init__(self, robot, item, arm):
        """
        Has the robot points to the object that was pointed to by the operator.

        This is based on the 'Grab' state, with difference that pickup is changed, i.e., without closing gripper.

        :param robot: Robot to use
        :param item: Designator that resolves to the item to grab. E.g. EntityByIdDesignator
        :param arm: Designator that resolves to the arm to use for grabbing. Eg. UnoccupiedArmDesignator
        :return:
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types
        states.check_type(item, Entity)
        states.check_type(arm, PublicArm)

        with self:
            smach.StateMachine.add('NAVIGATE_TO_POINT', states.NavigateToGrasp(robot, item, arm),
                                   transitions={'unreachable': 'RESET_FAILURE',
                                                'goal_not_defined': 'RESET_FAILURE',
                                                'arrived': 'PREPARE_POINT'})

            smach.StateMachine.add('PREPARE_POINT', states.PrepareEdGrasp(robot, arm, item),
                                   transitions={'succeeded': 'POINT',
                                                'failed': 'RESET_FAILURE'})

            smach.StateMachine.add('POINT', PointAt(robot, arm, item),
                                   transitions={'succeeded': 'done',
                                                'failed': 'RESET_FAILURE'})

            smach.StateMachine.add("RESET_FAILURE", states.ResetOnFailure(robot, arm),
                                   transitions={'done': 'failed'})


if __name__ == "__main__":

    rospy.init_node("test_grasping")

    robot = get_robot_from_argv(index=1)

    entity_id = "test_item"
    pose = FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0, 0, -1.57), kdl.Vector(2.6, -0.95, 0.8)), frame_id="/map")

    robot.ed.update_entity(id=entity_id, frame_stamped=pose)

    item = ds.EdEntityDesignator(robot, id=entity_id)

    arm = ds.UnoccupiedArmDesignator(robot, {})

    grab_state = IdentifyObject(robot, item, arm)
    grab_state.execute()
