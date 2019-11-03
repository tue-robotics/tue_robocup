import unittest

# datatypes
import PyKDL as kdl

# Robot Skills
from robot_skills.mockbot import Mockbot
from robot_skills.util.entity import Entity
from robot_skills.util.volume import BoxVolume

# Robot Smach States
from robot_smach_states.human_interaction.give_directions import get_room


class TestGetRoom(unittest.TestCase):

    def setUp(self):
        # simple rooms setup, all rooms are 1x1x3m
        # 1 -------   -----------   -------
        # | kitchen | living_room | hallway |
        # 0---------2-------------3---------4

        box1 = BoxVolume(kdl.Vector(0, 0, 0),
                         kdl.Vector(1, 1, 3))

        entity1 = Entity("kitchen", "room", "/map",
                             kdl.Frame(kdl.Rotation.RPY(0, 0, 0),
                                       kdl.Vector(0, 0, 0)),
                             None, {"in": box1}, None, 0)

        entity2 = Entity("living_room", "room", "/map",
                             kdl.Frame(kdl.Rotation.RPY(0, 0, 0),
                                       kdl.Vector(1, 0, 0)),
                             None, {"in": box1}, None, 0)

        entity3 = Entity("hallway", "room", "/map",
                             kdl.Frame(kdl.Rotation.RPY(0, 0, 0),
                                       kdl.Vector(2, 0, 0)),
                             None, {"in": box1}, None, 0)

        self.room_entities = [entity1, entity2, entity3]

    def test_get_room1(self):
        """
        Tests the 'get room' method
        """
        position = kdl.Vector(0.5, 0.5, 1.0)
        room = get_room(self.room_entities, position)
        self.assertEqual(room.id, 'kitchen')

    def test_get_room2(self):
        """
        Tests the 'get room' method
        """
        position = kdl.Vector(1.5, 0.5, 1.0)
        room = get_room(self.room_entities, position)
        self.assertEqual(room.id, 'living_room')

    def test_get_room_position_out_of_room(self):
        """
        Tests the 'get room' method with a position not in any room
        """
        position = kdl.Vector(3.5, 0.5, 1.0)
        with self.assertRaises(RuntimeError):
            get_room(self.room_entities, position)


if __name__ == '__main__':
    unittest.main()
