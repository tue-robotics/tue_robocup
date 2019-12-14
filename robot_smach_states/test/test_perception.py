import unittest

# datatypes
import PyKDL as kdl
from robot_skills.util.kdl_conversions import VectorStamped

# Robot Skills
from robot_skills.mockbot import Mockbot
from robot_skills.util.entity import Entity
from robot_skills.util.volume import BoxVolume
from robot_skills.util.shape import RightPrism

# Robot Smach States
import robot_smach_states as states
import robot_smach_states.util.designators as ds


class TestLookAtEntity(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def setUp(self):
        self.entity = Entity("12345", "dummy", "/map",
                             kdl.Frame(kdl.Rotation.RPY(1, 0, 0),
                                       kdl.Vector(3, 3, 3)),
                             None, {}, None, 0)

    def test_look_at_enity_looks_at_correct_point(self):
        """Test that the robot looks at the center point of the named area, w.r.t. the frame of the entity"""
        entity_ds = ds.Designator(self.entity)

        state = states.LookAtEntity(self.robot, entity_ds, waittime=0)

        state.execute()

        vs = VectorStamped(0, 0, 0, "/12345")

        self.robot.head.look_at_point.assert_called_with(vs)


class TestLookAtArea(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def setUp(self):
        box = BoxVolume(kdl.Vector(0, 0, 0),
                        kdl.Vector(1, 1, 1))

        self.entity = Entity("12345", "dummy", "/map",
                             kdl.Frame(kdl.Rotation.RPY(1, 0, 0),
                                       kdl.Vector(3, 3, 3)),
                             None, {"dummy_volume": box}, None, 0)

        self.area = "dummy_volume"

    def test_look_at_area_looks_at_correct_point(self):
        """Test that the robot looks at the center point of the named area, w.r.t. the frame of the entity"""
        entity_ds = ds.Designator(self.entity)
        area_ds = ds.Designator(self.area)

        state = states.LookAtArea(self.robot, entity_ds, area_ds, waittime=0)

        state.execute()

        vs = VectorStamped(0.5, 0.5, 0.5, "/12345")

        self.robot.head.look_at_point.assert_called_with(vs, timeout=0)


if __name__ == '__main__':
    unittest.main()
