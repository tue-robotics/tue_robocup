import sys
import unittest

from robocup_knowledge.knowledge_loader import load_knowledge


class TestKnowledgeLoader(unittest.TestCase):
    def test_load_knowledge(self):
        knowledge = load_knowledge("challenge_stickler_for_the_rules")
        required_keys = ["common", "starting_point", "waypoint_ids", "forbidden_room", "forbidden_room_waypoint",
                         "drinks_entity_id"]
        required_common_keys = ["female_names", "male_names", "names", "locations", "location_rooms",
                                "location_categories", "location_names", "manipulation_locations", "objects",
                                "object_names", "object_categories", "object_color", "object_size", "object_weight",
                                "category_locations", "inspect_areas", "inspect_positions", "default_target_radius",
                                "most_probable_location_in_room_map", "get_location_from_room",
                                "object_names_of_category", "drink_names", "dn", "drink_spec", "bcolors", "make_prints",
                                "is_location", "get_room", "is_room", "get_inspect_areas", "get_inspect_position",
                                "is_pick_location", "is_place_location", "get_locations", "is_known_object",
                                "get_objects", "get_object_category", "get_object_color", "get_object_size",
                                "get_object_weight", "get_object_category_location"]

        for key in required_keys:
            self.assertIn(key, knowledge.__dict__, msg=f"{key} is not in knowledge: {knowledge}")

        for key in required_common_keys:
            self.assertIn(key, knowledge.common.__dict__, msg=f"{key} is not in the common knowledge {knowledge.common}")

if __name__ == "__main__":
    unittest.main()
