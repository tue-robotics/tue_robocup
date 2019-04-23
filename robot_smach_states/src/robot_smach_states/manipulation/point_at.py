# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills.arms import Arm
from robot_skills.util.entity import Entity
from robot_smach_states.util.designators import check_type


class PointAt(smach.State):
    def __init__(self, robot, arm_designator, point_at_designator, look_at_designator=None):
        """
        Drive the robot back a little and move the designated arm to place the designated item at the designated pose
        :param robot: Robot to execute state with
        :param placement_pose: Designator that resolves to the pose to place at. E.g. an EmptySpotDesignator
        :param arm: Designator -> arm to place with, so Arm that holds entity_to_place, e.g. via
        ArmHoldingEntityDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        if look_at_designator is None:
            look_at_designator = point_at_designator

        # Check types or designator resolve types
        check_type(arm_designator, Arm)
        check_type(point_at_designator, Entity)
        check_type(look_at_designator, Entity)

        # Assign member variables
        self._robot = robot
        self._arm_designator = arm_designator
        self._point_at_designator = point_at_designator
        self._look_at_designator = look_at_designator

    def execute(self, userdata=None):
        point_at_ent = self._point_at_designator.resolve()  # type: Entity
        if not point_at_ent:
            rospy.logerr("Could not resolve _point_at_designator")
            return "failed"

        if self._look_at_designator != self._point_at_designator:
            look_at_ent = self._look_at_designator.resolve()  # type: Entity
            if not look_at_ent:
                rospy.logerr("Could not resolve _look_at_designator")
                return "failed"
        else:
            look_at_ent = point_at_ent  # type: Entity

        arm = self._arm_designator.resolve()  # type: Arm
        if not arm:
            rospy.logerr("Could not resolve _arm_designator")
            return "failed"

        # TODO: make arm point at some pose
        arm.point_at(point_at_ent.pose.extractVectorStamped())
        # Arm to position in a safe way
        # arm.send_joint_trajectory('handover_to_human', timeout=0)  # TODO define pointing pose
        arm.wait_for_motion_done()

        self.robot.head.look_at_point(look_at_ent.pose.extractVectorStamped())

        return 'succeeded'
