import unittest
from types import SimpleNamespace

from robot_smach_states.util.entity_descriptions import description_satisfies, entity_matches

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


if __name__ == "__main__":
    unittest.main()
