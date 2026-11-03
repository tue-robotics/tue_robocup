"""
Grasp with a Vision-Language-Action policy instead of the geometric grasp pipeline in grab.py

The policy server (see robot_skills.vla_policy) is queried in a loop: observe, predict a chunk, execute the
first part of the chunk, repeat. Arm deltas are integrated into a timed joint trajectory, base velocities are
streamed to the base controller and the gripper is toggled on a threshold. The head is held still: the policy
was trained with a fixed, downward looking head.

Run with execute=False first ("shadow mode"): the loop runs and the predicted chunks are logged and published
for inspection, but nothing is commanded.
"""
from __future__ import absolute_import

from typing import Optional, Union

# ROS
import rospy
import smach
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# TU/e Robotics
from ed.entity import Entity
from robot_skills.robot import Robot
from robot_skills.arm.arms import GripperTypes
from robot_skills.vla_chunk import ChunkLimits, ChunkPlan, gripper_command, group_max_abs, plan_chunk
from robot_skills.vla_policy import VLAPolicyError
from ..util.designators import check_type
from ..util.designators.arm import ArmDesignator
from ..util.designators.core import Designator


class VLAGrasp(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING], }

    def __init__(self, robot: Robot, arm: ArmDesignator, instruction: Union[Designator, str],
                 grab_entity: Optional[Designator] = None, execute: bool = False, max_duration: float = 30.0,
                 steps_per_chunk: int = 15, limits: Optional[ChunkLimits] = None, lift_to_succeed: float = 0.05) -> None:
        """
        Grasp an object by running a VLA policy in closed loop

        :param robot: robot to execute this state with, must have a 'vla' part
        :param arm: Designator that resolves to the arm to grasp with
        :param instruction: Designator that resolves to the instruction for the policy, e.g. "pick up the mug"
        :param grab_entity: Optional designator that resolves to the entity being grasped. On success it is
            registered as occupying the gripper so that subsequent states (e.g. Place) can use it.
        :param execute: Whether to command the robot. When False (shadow mode) the predicted chunks are only
            logged and published on <robot>/vla/predicted_trajectory.
        :param max_duration: time [s] after which the state returns 'timeout'
        :param steps_per_chunk: number of rows of every predicted chunk to execute before predicting again
        :param limits: rate and safety limits, see robot_skills.vla_chunk.ChunkLimits
        :param lift_to_succeed: rise of the arm lift joint [m] after closing the gripper that counts as a successful grasp
        """
        smach.State.__init__(self, outcomes=['succeeded', 'timeout', 'failed'])

        self._robot = robot
        self._arm_designator = arm
        self._instruction = instruction
        self._grab_entity_designator = grab_entity
        self._execute = execute
        self._max_duration = max_duration
        self._steps_per_chunk = steps_per_chunk
        self._limits = limits if limits is not None else ChunkLimits()
        self._lift_to_succeed = lift_to_succeed

        check_type(instruction, str)
        if grab_entity is not None:
            check_type(grab_entity, Entity)

        self._debug_pub = rospy.Publisher("/" + robot.robot_name + "/vla/predicted_trajectory", JointTrajectory,
                                          queue_size=1)

        assert self._robot.get_arm(**self.REQUIRED_ARM_PROPERTIES) is not None, \
            "None of the available arms meets all this class's requirements: {}".format(self.REQUIRED_ARM_PROPERTIES)

    def execute(self, userdata=None) -> str:
        arm = self._arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return 'failed'

        instruction = self._instruction.resolve() if hasattr(self._instruction, "resolve") else self._instruction
        if not instruction:
            rospy.logerr("Could not resolve instruction")
            return 'failed'

        vla = self._robot.vla
        limits = self._limits
        mode = "executing" if self._execute else "shadow mode"
        rospy.loginfo("VLAGrasp '%s' (%s, %s)", instruction, mode, vla.server_url)

        if self._execute:
            self._robot.base.local_planner.cancelCurrentPlan()

        rate = rospy.Rate(limits.fps)
        end_time = rospy.Time.now() + rospy.Duration(self._max_duration)
        gripper_state = None
        lift_at_close = None
        outcome = 'timeout'

        try:
            while rospy.Time.now() < end_time and not rospy.is_shutdown():
                try:
                    start = rospy.Time.now()
                    chunk = vla.predict(instruction)
                    state = vla.get_state()
                    latency = (rospy.Time.now() - start).to_sec()
                except VLAPolicyError as e:
                    rospy.logerr("VLAGrasp: %s", e)
                    return 'failed'

                plan = plan_chunk(chunk, state[:5], limits, self._steps_per_chunk)
                rospy.loginfo("VLA chunk after %.0f ms, max |action| per group: %s", latency * 1000,
                              {k: round(v, 3) for k, v in group_max_abs(chunk).items()})
                self._publish_plan(arm, plan)

                # A successful grasp: the gripper closed and the arm went up afterwards
                if lift_at_close is not None and state[0] - lift_at_close > self._lift_to_succeed:
                    outcome = 'succeeded'
                    break

                # The gripper acts on the last value of the prefix that is executed
                command = gripper_command(float(plan.gripper[-1]), limits, gripper_state)
                if command:
                    gripper_state = command
                    lift_at_close = state[0] if command == "close" else None
                    rospy.loginfo("VLA gripper: %s", command)
                    if self._execute:
                        arm.gripper.send_goal(command, timeout=0.0)

                if self._execute:
                    arm.send_joint_positions_timed(plan.arm_positions.tolist(), plan.times_from_start.tolist())
                    for vx, vy, vth in plan.base_velocities:
                        self._robot.base.send_velocity(vx, vy, vth)
                        rate.sleep()
                else:
                    rospy.sleep(plan.times_from_start[-1])
        finally:
            if self._execute:
                self._robot.base.stop()

        if outcome == 'succeeded':
            # Same bookkeeping as PickUp so that subsequent states (Place, HandOver) find the object in the gripper
            if self._grab_entity_designator is not None:
                entity = self._grab_entity_designator.resolve()
                if entity:
                    arm.gripper.occupied_by = entity
                    # Remove pose from ED as we are holding the object in the gripper
                    self._robot.ed.update_entity(uuid=entity.uuid, remove_pose=True)
            if arm.has_joint_goal('carrying_pose'):
                arm.send_joint_goal('carrying_pose', timeout=0.0)
            self._robot.head.cancel_goal()
        rospy.loginfo("VLAGrasp: %s", outcome)
        return outcome

    def _publish_plan(self, arm, plan: ChunkPlan) -> None:
        """Publish the planned arm trajectory so it can be inspected, e.g. in shadow mode"""
        trajectory = JointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.joint_names = self._robot.vla.arm_joint_names
        trajectory.points = [JointTrajectoryPoint(positions=positions.tolist(), time_from_start=rospy.Duration.from_sec(t))
                             for positions, t in zip(plan.arm_positions, plan.times_from_start)]
        self._debug_pub.publish(trajectory)
