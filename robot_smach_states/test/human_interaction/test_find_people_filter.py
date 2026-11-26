"""
How FindPeople selects people, with the filters the challenges use. These pin the behaviour of receptionist,
stickler_for_rules and restaurant now that the matching also serves the descriptions of the task planner.
"""
import unittest
from types import SimpleNamespace

from robot_smach_states.human_interaction.find_people_in_room import check_person_property, filter_people


class Person(object):
    """A person entity as far as FindPeople uses it; hashable like ed_python's Entity"""
    def __init__(self, uuid, person_properties):
        self.uuid = uuid
        self.person_properties = person_properties


def person(uuid, tags=(), posture="", shirt_colors=(), name="", properties=True):
    props = SimpleNamespace(tags=list(tags), posture=posture, shirt_colors=list(shirt_colors), name=name) \
        if properties else None
    return Person(uuid, props)


WAVING_LEFT = person("waving-left", tags=["LWaving", "RNotHolding"])
WAVING_RIGHT = person("waving-right", tags=["RWaving"])
WAVING_BOTH = person("waving-both", tags=["RWaving", "LWaving"])
HOLDING = person("holding", tags=["LHolding", "RNotHolding"])
SITTING_RED = person("sitting-red", tags=["LSitting"], posture="sitting", shirt_colors=["red", "black"])
ANNA = person("anna", name="anna")
NO_PROPERTIES = person("unknown", properties=False)
EVERYONE = [WAVING_LEFT, WAVING_RIGHT, WAVING_BOTH, HOLDING, SITTING_RED, ANNA, NO_PROPERTIES]


def uuids(people):
    return sorted(p.uuid for p in people)


class TestChallengeFilters(unittest.TestCase):
    def test_receptionist_takes_everyone(self):
        # introduce_guest.py: FindPeople without properties
        self.assertEqual(uuids(EVERYONE), uuids(filter_people(EVERYONE)))

    def test_restaurant_takes_people_waving_with_either_arm(self):
        # restaurant.py: properties={'tags': ['LWaving', 'RWaving']}, strict=False
        self.assertEqual(["waving-both", "waving-left", "waving-right"],
                         uuids(filter_people(EVERYONE, {"tags": ["LWaving", "RWaving"]}, strict=False)))

    def test_stickler_takes_people_not_holding_anything(self):
        # patrol.py: properties={'tags': ['LHolding', 'RHolding']}, strict=False, reverse=True
        remaining = filter_people(EVERYONE, {"tags": ["LHolding", "RHolding"]}, strict=False, reverse=True)
        self.assertNotIn("holding", uuids(remaining))
        self.assertIn("waving-left", uuids(remaining))

    def test_stickler_wave_tags_are_unknown_to_perception(self):
        # patrol.py asks for 'LWave'/'RWave', which perception never gives ('LWaving'): nobody matches, as before
        self.assertEqual([], filter_people(EVERYONE, {"tags": ["LWave", "RWave"]}, strict=False))


class TestCheckPersonProperty(unittest.TestCase):
    def test_strict_needs_all_values_in_any_order(self):
        # Before, a strict check depended on the order of the person's tags
        self.assertTrue(check_person_property(WAVING_BOTH, "tags", ["LWaving", "RWaving"], strict=True))
        self.assertFalse(check_person_property(WAVING_LEFT, "tags", ["LWaving", "RWaving"], strict=True))

    def test_case_does_not_matter(self):
        # The action server lowercases the whole task
        self.assertTrue(check_person_property(WAVING_LEFT, "tags", ["lwaving"], strict=False))
        self.assertTrue(check_person_property(SITTING_RED, "posture", "SITTING", strict=True))

    def test_id_is_the_id_or_the_name(self):
        # Before, the 'id' key raised an error: person_properties has no 'id'
        self.assertTrue(check_person_property(ANNA, "id", "anna"))
        self.assertTrue(check_person_property(ANNA, "id", "Anna"))
        self.assertFalse(check_person_property(WAVING_LEFT, "id", "anna"))

    def test_a_person_without_properties_has_none(self):
        # Before, this raised an error
        self.assertFalse(check_person_property(NO_PROPERTIES, "tags", ["LWaving"], strict=False))

    def test_no_value_means_any_value(self):
        self.assertTrue(check_person_property(SITTING_RED, "posture", None))
        self.assertFalse(check_person_property(WAVING_LEFT, "posture", None))


class TestAttributes(unittest.TestCase):
    def test_waving(self):
        self.assertEqual(["waving-both", "waving-left", "waving-right"],
                         uuids(filter_people(EVERYONE, attributes={"gesture": "waving"})))

    def test_raising_the_left_arm(self):
        self.assertEqual(["waving-both", "waving-left"],
                         uuids(filter_people(EVERYONE, attributes={"gesture": "raising_left_arm"})))

    def test_pose_and_clothing_color(self):
        self.assertEqual(["sitting-red"],
                         uuids(filter_people(EVERYONE, attributes={"pose": "sitting", "clothing_color": "red"})))

    def test_an_uncheckable_attribute_does_not_filter(self):
        # Perception cannot see garments: only the gesture decides
        self.assertEqual(["waving-both", "waving-left", "waving-right"],
                         uuids(filter_people(EVERYONE, attributes={"gesture": "waving", "garment": "jacket"})))

    def test_properties_and_attributes_together(self):
        self.assertEqual(["anna"], uuids(filter_people([ANNA, WAVING_LEFT], {"id": "anna"})))


if __name__ == "__main__":
    unittest.main()
