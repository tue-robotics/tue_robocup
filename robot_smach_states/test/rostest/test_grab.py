#! /usr/bin/env python

import unittest
import rospy
import PyKDL as kdl
from pykdl_ros import FrameStamped

from ed.entity import Entity
from ed.shape import RightPrism

# Robot Skills
from robot_skills.mockbot import Mockbot

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
                            stamp=rospy.Time.now(),
                            frame_id="map")
        shape = RightPrism(
            [kdl.Vector(0, 0, 0), kdl.Vector(0, 0.05, 0), kdl.Vector(0.05, 0.05, 0), kdl.Vector(0.05, 0, 0)], -0.1, 0.1)
        item = Entity(entity_id, "test_type", pose.header.frame_id, pose.frame, shape, None, None, rospy.Time.now())

        itemdes = ds.Designator(item)
        arm = ds.UnoccupiedArmDesignator(self.robot).lockable()

        state = Grab(self.robot, itemdes, arm)
        self.assertEqual(state.execute(), "done")


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_grab')
    rostest.rosrun('robot_smach_states', 'test_grab', TestPickUp)
