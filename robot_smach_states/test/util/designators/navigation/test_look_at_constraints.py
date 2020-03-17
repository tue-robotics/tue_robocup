#! /usr/bin/env python
import unittest

from robot_smach_states.util.designators.navigation.pose_constraints import PoseConstraintsDesignator


class TestLookAtConstraintDesignator(unittest.TestCase):
    def test_resolve(self):
        n = PoseConstraintsDesignator(0.5, 1.3, rz=None, radius=0.15, frame_id="/map", name="pose designator")
        constr = n.resolve()
        self.assertIsNotNone(constr.pc)
        self.assertIsNone(constr.oc)

    def test_resolve_offset(self):
        n = PoseConstraintsDesignator(0.5, 1.3, rz=-1.57, radius=0.15, frame_id="/map", name="pose designator")
        constr = n.resolve()
        self.assertIsNotNone(constr.pc)
        self.assertIsNotNone(constr.oc)


if __name__ == '__main__':
    unittest.main()
