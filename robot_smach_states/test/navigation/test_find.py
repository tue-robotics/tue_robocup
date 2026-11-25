import unittest
from types import SimpleNamespace

import PyKDL as kdl

from robot_smach_states.navigation.find import entities_from_description


def entity(uuid, etype, x):
    """An ED entity as far as entities_from_description uses it"""
    return SimpleNamespace(uuid=uuid, etype=etype, distance_to_2d=lambda p, x=x: abs(x - p.x()))


class TestEntitiesFromDescription(unittest.TestCase):
    def setUp(self):
        entities = [entity("coke-1", "coke", 2.0),     # named by the detector
                    entity("drink-1", "drink", 1.0),   # the detector only told the category ("bottle")
                    entity("apple-1", "apple", 0.5),
                    entity("unknown-1", "", 0.1)]      # no type
        location = SimpleNamespace(frame=SimpleNamespace(p=kdl.Vector(0, 0, 0)))
        self.robot = SimpleNamespace(ed=SimpleNamespace(get_entities=lambda: entities),
                                     base=SimpleNamespace(get_location=lambda: location))
        categories = {"coke": "drink", "apple": "food"}
        self.knowledge = SimpleNamespace(get_object_category=lambda name: categories.get(name))

    def find(self, description, ids=None):
        return [e.uuid for e in entities_from_description(self.robot, self.knowledge, description, ids)]

    def test_category_matches_objects_of_it_and_entities_typed_as_it(self):
        self.assertEqual(["drink-1", "coke-1"], self.find({"category": "drink"}))

    def test_type_matches_only_that_type(self):
        # A named object does not match an entity that only has the category (no fallback)
        self.assertEqual(["coke-1"], self.find({"type": "coke"}))

    def test_restricted_to_the_segmented_entities(self):
        self.assertEqual(["coke-1"], self.find({"category": "drink"}, ["coke-1", "apple-1"]))


if __name__ == "__main__":
    unittest.main()
