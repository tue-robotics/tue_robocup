import unittest
import PyKDL as kdl

# Robot Skills
from robot_skills.mockbot import Mockbot, Base
from robot_skills.util.entity import Entity
from robot_skills.util.volume import BoxVolume
from robot_skills.util.kdl_conversions import FrameStamped

# Robot Smach States
from robot_smach_states.navigation.guidance import TourGuide, Guide


class TestTourGuide(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # simple rooms setup, all rooms are 5x5x3m
        # 5 -------  ---------
        # | kitchen | bedroom |
        # 0---------5---------10

        box1 = BoxVolume(kdl.Vector(0, 0, 0),
                         kdl.Vector(5, 5, 3))

        box2 = BoxVolume(kdl.Vector(0, 0, 0),
                         kdl.Vector(1, 1, 0.5))

        cls.robot = Mockbot()

        entity1 = Entity("kitchen", "room", "/map", kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0, 0, 0)), None,
                         {"in": box1}, None, 0)
        entity2 = Entity("bedroom", "room", "/map", kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(5, 0, 0)), None,
                         {"in": box1}, None, 0)
        entity3 = Entity("cabinet", "furniture", "/map", kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(4, 4, 0)),
                         None, {"in": box2}, None, 0)
        entity4 = Entity("bookcase", "furniture", "/map", kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(8, 1, 0)),
                         None, {"in": box2}, None, 0)

        cls.robot.ed._static_entities = {e.id: e for e in [entity1, entity2, entity3, entity4]}
        # cls._room_entities = [entity1, entity2]

    def test_get_room(self):
        position = kdl.Vector(3, 1, 0)
        tour_guide = TourGuide(self.robot)
        room = tour_guide.get_room(position)
        self.assertEqual(room.id, "kitchen")

    # def test_tour_guide(self):
    #     test = TourGuide(self._robot
    #     self.assertEqual(test.get_room(self._robot.base.get_location.frame.p), "kitchen")
    #     self.assertEqual(test.describe_near_objects(), "On our {}")


# class TestGuide(unittest.TestCase):
#     def setUp(self):
#         self.robot = Mockbot()
#
#     def test_guide(self):
#         test = Guide(self.robot)
