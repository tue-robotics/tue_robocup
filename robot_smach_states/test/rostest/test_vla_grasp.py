#! /usr/bin/env python

import unittest

import numpy as np
import PyKDL as kdl
import rospy
from pykdl_ros import FrameStamped

from ed.entity import Entity
from ed.shape import RightPrism

# Robot Skills
from robot_skills.mockbot import Mockbot
from robot_skills.vla_chunk import ChunkLimits
from robot_skills.vla_policy import VLAPolicyError

# Robot Smach States
from robot_smach_states.manipulation import Grab, VLAGrasp
from robot_smach_states.util import designators as ds


def closing_chunk():
    """A chunk that lifts the arm a little and closes the gripper"""
    chunk = np.zeros((50, 11), dtype=np.float32)
    chunk[:, 0] = 0.001
    chunk[:, 5] = -1.0
    chunk[:, 8] = 0.05
    return chunk


def test_entity(etype="coke"):
    """An entity with a shape and pose, as in test_grab.py"""
    pose = FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(0.0, 0.0, 0.0)),
                        stamp=rospy.Time.now(), frame_id="map")
    shape = RightPrism([kdl.Vector(0, 0, 0), kdl.Vector(0, 0.05, 0), kdl.Vector(0.05, 0.05, 0), kdl.Vector(0.05, 0, 0)],
                       -0.1, 0.1)
    return Entity("test_item", etype, pose.header.frame_id, pose.frame, shape, None, None, rospy.Time.now())


# The arm lift rises after the gripper has closed: close on the first chunk, lifted on the next
LIFTED_STATES = [[0.0] * 8, [0.1] + [0.0] * 7, [0.1] + [0.0] * 7, [0.1] + [0.0] * 7]


class TestVLAGrasp(unittest.TestCase):
    def setUp(self):
        self.robot = Mockbot()
        self.robot.vla.predict.return_value = closing_chunk()
        self.arm = ds.UnoccupiedArmDesignator(self.robot).lockable()
        self.limits = ChunkLimits(fps=30.0)

    def test_shadow_mode_does_not_command(self):
        state = VLAGrasp(self.robot, self.arm, ds.Designator("pick up the mug"), execute=False, max_duration=0.5,
                         steps_per_chunk=3, limits=self.limits)
        self.assertEqual(state.execute(), "timeout")
        self.robot.vla.predict.assert_called()
        arm = self.arm.resolve()
        # noinspection PyProtectedMember
        arm._arm.send_joint_positions_timed.assert_not_called()
        arm.gripper.send_goal.assert_not_called()
        self.robot.base.send_velocity.assert_not_called()

    def test_execute_succeeds_after_close_and_lift(self):
        # The arm lift rises after the gripper has closed
        self.robot.vla.get_state.side_effect = [[0.0] * 8, [0.0] * 8, [0.1] + [0.0] * 7, [0.1] + [0.0] * 7]
        state = VLAGrasp(self.robot, self.arm, ds.Designator("pick up the mug"), execute=True, max_duration=5.0,
                         steps_per_chunk=3, limits=self.limits, lift_to_succeed=0.05)
        self.assertEqual(state.execute(), "succeeded")
        arm = self.arm.resolve()
        arm.gripper.send_goal.assert_called_with("close", timeout=0.0)
        # noinspection PyProtectedMember
        arm._arm.send_joint_positions_timed.assert_called()
        self.robot.base.send_velocity.assert_called()
        self.robot.base.stop.assert_called()

    def test_execute_registers_entity_on_success(self):
        self.robot.vla.get_state.side_effect = LIFTED_STATES
        entity = test_entity()
        # Resolve before executing: once the gripper is occupied, an unoccupied arm designator resolves to another arm
        arm = self.arm.resolve()
        state = VLAGrasp(self.robot, self.arm, ds.Designator("pick up the mug"), grab_entity=ds.Designator(entity),
                         execute=True, max_duration=5.0, steps_per_chunk=3, limits=self.limits)
        self.assertEqual(state.execute(), "succeeded")
        self.assertEqual(arm.gripper.occupied_by, entity)
        self.robot.ed.update_entity.assert_called_with(uuid=entity.uuid, remove_pose=True)
        self.robot.head.cancel_goal.assert_called()

    def test_grab_off_has_no_vla_state(self):
        sm = Grab(self.robot, ds.Designator(test_entity()), self.arm, vla_mode="off")
        self.assertNotIn("VLA_GRASP", sm.get_children())
        self.assertIn("GRAB", sm.get_children())

    def test_grab_primary_grasps_with_vla(self):
        self.robot.vla.get_state.side_effect = LIFTED_STATES
        sm = Grab(self.robot, ds.Designator(test_entity("coke")), self.arm, vla_mode="primary")
        self.assertIn("VLA_GRASP", sm.get_children())
        self.assertNotIn("GRAB", sm.get_children())
        self.assertEqual(sm.execute(), "done")
        self.robot.vla.predict.assert_called_with("pick up the coke")

    def test_grab_fallback_routes_grab_failure_to_vla(self):
        sm = Grab(self.robot, ds.Designator(test_entity()), self.arm, vla_mode="fallback")
        self.assertEqual(sm._transitions["GRAB"]["failed"], "VLA_GRASP")
        self.assertEqual(sm._transitions["VLA_GRASP"]["succeeded"], "done")
        # The geometric grasp succeeds on the mockbot, so the VLA is never consulted
        self.assertEqual(sm.execute(), "done")
        self.robot.vla.predict.assert_not_called()

    def test_server_error_fails(self):
        self.robot.vla.predict.side_effect = VLAPolicyError("no server")
        state = VLAGrasp(self.robot, self.arm, ds.Designator("pick up the mug"), execute=True, max_duration=5.0,
                         limits=self.limits)
        self.assertEqual(state.execute(), "failed")
        self.robot.base.stop.assert_called()


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_vla_grasp')
    rostest.rosrun('robot_smach_states', 'test_vla_grasp', TestVLAGrasp)
