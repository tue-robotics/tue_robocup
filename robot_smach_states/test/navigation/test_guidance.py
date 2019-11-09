import unittest
import PyKDL as kdl
import re

# Robot Skills
from robot_skills.mockbot import Mockbot
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

        cls._kitchen = Entity("kitchen", "room", "/map", kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0, 0, 0)),
                              None, {"in": box1}, ["room"], 0)
        cls._bedroom = Entity("bedroom", "room", "/map", kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(5, 0, 0)),
                              None, {"in": box1}, ["room"], 0)
        cls._cabinet = Entity("cabinet", "furniture", "/map", kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(4, 4, 0)),
                              None, {"on_top_off": box2}, ["furniture"], 0)
        cls._bookcase = Entity("bookcase", "furniture", "/map", kdl.Frame(kdl.Rotation.RPY(0, 0, 0),
                                                                          kdl.Vector(8, 1, 0)),
                               None, {"on_top_off": box2}, ["furniture"], 0)

        cls.robot.ed._static_entities = {e.id: e for e in [cls._kitchen, cls._bedroom, cls._cabinet, cls._bookcase]}

        cls.tour_guide = TourGuide(cls.robot)

    def setUp(self):
        self.robot.base.get_location = lambda: FrameStamped(kdl.Frame(kdl.Rotation().Identity(), kdl.Vector(-1, -1, 0)),
                                                            "/map")
        self.tour_guide.initialize()

    def assertEqualEllipsis(self, first, second, ellipsis_marker='...', msg=None):
        """
        Example :
            >>> self.assertEqualEllipsis('foo123bar', 'foo...bar')
        """
        if ellipsis_marker not in second:
            return first == second

        if re.match(re.escape(second).replace(re.escape(ellipsis_marker), '(.*?)'), first, re.M | re.S) is None:
            self.assertMultiLineEqual(first, second, msg)

    def test_get_room(self):
        room = self.tour_guide.get_room(kdl.Vector(3, 1, 0))
        self.assertEqual(room.id, "kitchen")

    def test_get_room_raise(self):
        position = kdl.Vector(20, 20, 0)
        with self.assertRaises(RuntimeError) as cm:
            self.tour_guide.get_room(position)

        self.assertEqualEllipsis(str(cm.exception), "Position ... is not in any room")

    def test_reset(self):
        self.tour_guide._passed_furniture_ids = ["entity1", "entity2"]
        self.tour_guide._passed_room_ids = ["entity3", "entity4"]
        self.tour_guide.reset()
        self.assertEqual(self.tour_guide._passed_furniture_ids, [])
        self.assertEqual(self.tour_guide._passed_room_ids, [])

    def test_initialize(self):
        # initialize is run in setUp, but robot.base.get_location has been replaced, so running it again
        self.robot.base.get_location = lambda: FrameStamped(kdl.Frame(kdl.Rotation().Identity(),
                                                                      kdl.Vector(3.5, 3.5, 0)), "/map")
        self.tour_guide.initialize()
        self.assertListEqual(self.tour_guide._passed_room_ids, [self._kitchen.id])
        self.assertListEqual(self.tour_guide._passed_furniture_ids, [self._cabinet.id])
        self.assertListEqual(sorted(self.tour_guide._furniture_entities), sorted([self._cabinet, self._bookcase]))
        self.assertListEqual(sorted(self.tour_guide._room_entities), sorted([self._kitchen, self._bedroom]))
        self.assertDictEqual(self.tour_guide._furniture_entities_room, {self._kitchen: [self._cabinet],
                                                                        self._bedroom: [self._bookcase]})

    def test_initialize2(self):
        self.robot.base.get_location = lambda: FrameStamped(kdl.Frame(kdl.Rotation().Identity(), kdl.Vector(20, 20, 2)),
                                                            "/map")
        self.tour_guide.initialize()
        self.assertListEqual(self.tour_guide._passed_room_ids, [])

    def test_describe_near_objects(self):
        self.robot.base.get_location = lambda: FrameStamped(kdl.Frame(kdl.Rotation().Identity(), kdl.Vector(1, 1, 0)),
                                                            "/map")
        self.assertEqual("We now enter the kitchen", self.tour_guide.describe_near_objects())

    def test_describe_near_objects2(self):
        self.robot.base.get_location = lambda: FrameStamped(kdl.Frame(kdl.Rotation().Identity(), kdl.Vector(6, 4, 0)),
                                                            "/map")
        self.assertEqual("We now enter the bedroom", self.tour_guide.describe_near_objects())

    def test_describe_near_objects3(self):
        self.assertEqual("", self.tour_guide.describe_near_objects())

    def test_describe_near_objects4(self):
        self.robot.base.get_location = lambda: FrameStamped(kdl.Frame(kdl.Rotation().Identity(), kdl.Vector(7.5, 2, 0)),
                                                            "/map")
        self.assertEqual("We now enter the bedroom", self.tour_guide.describe_near_objects())
        self.assertEqual("On our right you can see the bookcase", self.tour_guide.describe_near_objects())
        self.assertEqual("", self.tour_guide.describe_near_objects())


