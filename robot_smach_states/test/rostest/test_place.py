#! /usr/bin/env python

import unittest
import rospy
import PyKDL as kdl
from pykdl_ros import FrameStamped

# Robot Skills
from robot_skills.mockbot import Mockbot
from robot_skills.util.entity import Entity
from robot_skills.util.shape import RightPrism

# Robot Smach States
from robot_smach_states.manipulation import Place
from robot_smach_states.util import designators as ds


class TestPlace(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        self.robot = Mockbot()

    def test_place(self):
        entity_id = "test_item"
        pose = FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(0.0, 0.0, 0.0)),
                            stamp=rospy.Time.now(),
                            frame_id="map")
        shape = RightPrism(
            [kdl.Vector(0, 0, 0), kdl.Vector(0, 0.05, 0), kdl.Vector(0.05, 0.05, 0), kdl.Vector(0.05, 0, 0)], -0.1, 0.1)
        item = Entity(entity_id, "test_type", None, None, shape, None, None, rospy.Time.now())

        item_des = ds.Designator(item)
        arm = ds.ArmDesignator(self.robot).lockable()

        pose_des = ds.Designator(pose)

        state = Place(self.robot, item_des, pose_des, arm)
        self.assertEqual(state.execute(), "done")


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_place')
    rostest.rosrun('robot_smach_states', 'test_place', TestPlace)
