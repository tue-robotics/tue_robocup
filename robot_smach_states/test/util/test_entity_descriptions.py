import unittest
from types import SimpleNamespace

from robot_smach_states.util.entity_descriptions import PERSON_ATTRIBUTES, description_satisfies, \
    entity_matches, person_matches

KNOWLEDGE = SimpleNamespace(get_object_category={"coke": "drink", "water": "drink", "apple": "food"}.get)


class TestEntityMatches(unittest.TestCase):
    def matches(self, entity_type, description):
        return entity_matches(entity_type, description, KNOWLEDGE)

    def test_a_type_matches_only_that_type(self):
        self.assertTrue(self.matches("coke", {"type": "coke"}))
        self.assertFalse(self.matches("water", {"type": "coke"}))
        # An entity the detector only knows as a drink may be another drink: no fallback to the category
        self.assertFalse(self.matches("drink", {"type": "coke"}))

    def test_a_category_matches_its_objects_and_entities_typed_as_it(self):
        self.assertTrue(self.matches("coke", {"category": "drink"}))
        self.assertTrue(self.matches("drink", {"category": "drink"}))
        self.assertFalse(self.matches("apple", {"category": "drink"}))

    def test_an_entity_without_type_matches_nothing(self):
        self.assertFalse(self.matches("", {"category": "drink"}))
        self.assertFalse(self.matches(None, {"type": "coke"}))

    def test_a_description_without_type_or_category_matches_nothing(self):
        self.assertFalse(self.matches("coke", {}))
        self.assertFalse(self.matches("coke", {"type": None, "category": None}))

    def test_case_does_not_matter(self):
        # The action server lowercases the whole task
        self.assertTrue(self.matches("Coke", {"type": "coke"}))
        self.assertTrue(self.matches("coke", {"category": "DRINK"}))

    def test_type_wins_over_category(self):
        self.assertFalse(self.matches("water", {"type": "coke", "category": "drink"}))

    def test_unknown_objects_have_no_category(self):
        self.assertFalse(self.matches("laptop", {"category": "drink"}))

    def test_descriptions_may_be_objects(self):
        # The action server's EntityDescription has the same attributes
        self.assertTrue(self.matches("coke", SimpleNamespace(type=None, category="drink")))


class TestDescriptionSatisfies(unittest.TestCase):
    def satisfies(self, held, wanted):
        return description_satisfies(held, wanted, KNOWLEDGE)

    def test_the_same_type(self):
        self.assertTrue(self.satisfies({"type": "coke"}, {"type": "coke"}))
        self.assertFalse(self.satisfies({"type": "water"}, {"type": "coke"}))

    def test_a_type_satisfies_its_category(self):
        self.assertTrue(self.satisfies({"type": "coke"}, {"category": "drink"}))
        self.assertFalse(self.satisfies({"type": "coke"}, {"category": "food"}))

    def test_a_category_satisfies_the_same_category(self):
        self.assertTrue(self.satisfies({"category": "drink"}, {"category": "drink"}))
        self.assertFalse(self.satisfies({"category": "drink"}, {"category": "food"}))

    def test_a_category_does_not_satisfy_a_type(self):
        # The held drink may be water when a coke is asked for: fetch the coke
        self.assertFalse(self.satisfies({"category": "drink"}, {"type": "coke"}))

    def test_empty_descriptions_satisfy_nothing(self):
        # Before, two missing types compared equal (None == None) and a held drink counted as the requested food
        self.assertFalse(self.satisfies({"type": None, "category": None}, {"type": None, "category": None}))
        self.assertFalse(self.satisfies(None, {"type": "coke"}))
        self.assertFalse(self.satisfies({"type": "coke"}, None))
        self.assertFalse(self.satisfies({"type": "coke"}, {}))

    def test_held_type_wins_over_held_category(self):
        self.assertTrue(self.satisfies({"type": "coke", "category": "drink"}, {"type": "coke"}))


def props(tags=(), posture="", shirt_colors=()):
    return SimpleNamespace(tags=list(tags), posture=posture, shirt_colors=list(shirt_colors))


class TestPersonMatches(unittest.TestCase):
    def test_every_gesture(self):
        expected = {"waving": ["LWaving"], "raising_left_arm": ["LWaving"], "raising_right_arm": ["RWaving"],
                    "pointing_left": ["LPointing", "PointingLeft"], "pointing_right": ["PointingRight"]}
        self.assertEqual(set(PERSON_ATTRIBUTES["gesture"]), set(expected))
        for gesture, tags in expected.items():
            self.assertEqual((True, []), person_matches(props(tags), {"gesture": gesture}), gesture)
            self.assertEqual((False, []), person_matches(props(["RNotHolding"]), {"gesture": gesture}), gesture)

    def test_raising_one_arm_is_not_the_other(self):
        self.assertFalse(person_matches(props(["LWaving"]), {"gesture": "raising_right_arm"})[0])

    def test_pose_from_the_posture_or_the_tags(self):
        self.assertTrue(person_matches(props(posture="standing"), {"pose": "standing"})[0])
        self.assertFalse(person_matches(props(posture="sitting"), {"pose": "standing"})[0])
        self.assertTrue(person_matches(props(["RLaying"]), {"pose": "lying"})[0])  # no posture measured

    def test_standing_without_a_posture_is_unchecked(self):
        self.assertEqual((True, ["pose"]), person_matches(props(), {"pose": "standing"}))

    def test_clothing_color(self):
        self.assertTrue(person_matches(props(shirt_colors=["Red", "black"]), {"clothing_color": "red"})[0])
        self.assertTrue(person_matches(props(shirt_colors=["grey"]), {"clothing_color": "gray"})[0])
        self.assertFalse(person_matches(props(shirt_colors=["blue"]), {"clothing_color": "red"})[0])

    def test_garment_is_unchecked(self):
        self.assertEqual((True, ["garment"]), person_matches(props(), {"garment": "jacket"}))

    def test_a_person_without_properties_does_not_fit(self):
        self.assertEqual((False, []), person_matches(None, {"gesture": "waving"}))
        self.assertEqual((True, ["garment"]), person_matches(None, {"garment": "jacket"}))

    def test_none_values_are_ignored(self):
        self.assertEqual((True, []), person_matches(props(), {"gesture": None, "pose": None}))


if __name__ == "__main__":
    unittest.main()
