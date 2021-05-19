#! /usr/bin/env python

import unittest
import rospy
import PyKDL as kdl

# Robot Skills
from robot_skills.mockbot import Mockbot
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import FrameStamped
from robot_skills.util.shape import RightPrism

# Robot Smach States
from robot_smach_states.manipulation import Grab
from robot_smach_states.util import designators as ds


class TestPickUp(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        self.robot = Mockbot()

    def test_grab(self):
        entity_id = "test_item"
        pose = FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(0.0, 0.0, 0.0)),
                            frame_id="map")
        self.robot.ed.update_entity(id=entity_id, frame_stamped=pose)
        shape = RightPrism(
            [kdl.Vector(0, 0, 0), kdl.Vector(0, 0.05, 0), kdl.Vector(0.05, 0.05, 0), kdl.Vector(0.05, 0, 0)], -0.1, 0.1)
        item = Entity(entity_id, "test_type", pose.frame_id, pose.frame, shape, None, None, None)

        itemdes = ds.Designator(item)
        arm = ds.UnoccupiedArmDesignator(self.robot).lockable()

        state = Grab(self.robot, itemdes, arm)
        self.assertEqual(state.execute(), "succeeded")


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_grab')
    rostest.rosrun('robot_smach_states', 'test_grab', TestPickUp)
