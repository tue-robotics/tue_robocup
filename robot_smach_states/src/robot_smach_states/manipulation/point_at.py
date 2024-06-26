from __future__ import absolute_import, print_function

# ROS
from pykdl_ros import VectorStamped
import rospy
import smach

# TU/e Robotics
from ed.entity import Entity
from robot_skills.arm.arms import PublicArm
from robot_smach_states.util.designators import check_type

# System
import numpy as np
import sys


class PointAt(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_goals": ['point_at'], }

    def __init__(self, robot, arm_designator, point_at_designator, look_at_designator=None):
        """
        Drive the robot back a little and move the designated arm to place the designated item at the designated pose

        :param robot: Robot to execute state with
        :param arm_designator : arm to place with, so Arm that holds entity_to_place, E.g. ArmHoldingEntityDesignator
        :param point_at_designator: Designator that resolves to the pose to place at. E.g. EmptySpotDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        if look_at_designator is None:
            look_at_designator = point_at_designator

        # Check types or designator resolve types
        check_type(arm_designator, PublicArm)
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
        arm = self._arm_designator.resolve()  # type: PublicArm
        if not arm:
            rospy.logerr("Could not resolve _arm_designator")
            return "failed"
        # TODO: make arm point at some pose
        vs = VectorStamped.from_framestamped(point_at_ent.pose)
        vector_in_bs = self._robot.tf_buffer.transform(vs, self._robot.base_link_frame, timeout=rospy.Duration(1.0))
        # tan(angle) = dy / dx
        # angle = arctan(dy / dx)
        # Arm to position in a safe way
        rotate_base = np.arctan2(vector_in_bs.vector.y(), vector_in_bs.vector.x())  # Radians
        # For 1 second, rotate the base with vth == rotate_base.
        # vth is in radians/sec but we rotate for 1 s to that should equal $rotate_base in the end.

        maxvel = np.pi / 10.0  # Max rotation speed in rad/s
        duration = abs(rotate_base / maxvel)  # duration of rotation, in s
        vel = maxvel * np.sign(rotate_base)
        rospy.loginfo("Rotate base by {:.3f}deg. At {:.3f}deg/s this takes {}s".format(np.degrees(rotate_base),
                                                                                       np.degrees(vel),
                                                                                       duration))
        self._robot.base.force_drive(0, 0, vel, duration)

        arm.send_joint_goal('point_at', timeout=0)
        arm.wait_for_motion_done()

        self._robot.head.look_at_point(VectorStamped.from_framestamped(look_at_ent.pose))
        self._robot.head.wait_for_motion_done()

        return 'succeeded'


class PointAtSM(smach.StateMachine):

    def __init__(self, robot, arm_designator, point_at_designator, look_at_designator=None):
        """
        Drive the robot back a little and move the designated arm to place the designated item at the designated pose

        :param robot: Robot to execute state with
        :param arm_designator : arm to place with, so Arm that holds entity_to_place, E.g. ArmHoldingEntityDesignator
        :param point_at_designator: Designator that resolves to the pose to place at. E.g. EmptySpotDesignator
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        with self:
            smach.StateMachine.add('RESOLVE_ARM', ResolveArm(arm_designator, self),
                                   transitions={'succeeded': 'POINT_AT',
                                                'failed': 'failed'})

            smach.StateMachine.add('POINT_AT', PointAt(robot, arm_designator, point_at_designator),
                                   transitions={'failed': 'failed',
                                                'succeeded': 'done'})

        # check_arm_requirements(self, robot)


if __name__ == "__main__":
    from robot_skills import get_robot
    from robot_smach_states.util.designators import EdEntityDesignator, UnoccupiedArmDesignator
    from robot_smach_states.utility import ResolveArm

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
        point_at = sys.argv[2]
        look_at = sys.argv[3]

        rospy.init_node('test_follow_operator')
        robot = get_robot(robot_name)
        sm = PointAtSM(robot,
                       arm_designator=UnoccupiedArmDesignator(robot).lockable(),
                       point_at_designator=EdEntityDesignator(robot, uuid=point_at, name='point_at_des'),
                       look_at_designator=EdEntityDesignator(robot, uuid=look_at, name='look_at_des'))
        sm.execute()
    else:
        print("Please provide robot name, point_at ID and look_at ID as argument.")
        exit(1)
