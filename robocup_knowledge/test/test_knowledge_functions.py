# System
import unittest

# RoboCup Knowledge
from robocup_knowledge import knowledge_functions


class TestKnowledgeFunctions(unittest.TestCase):
    # N.B.: test cases based on current impuls common knowledge. Needs to be copied to a separate location
    def test_knowledge_functions(self) -> None:
        print("Hello World!")
        print(f"Names: {knowledge_functions.names}")
        print(f"Locations: {knowledge_functions.locations}")
        print(f"Location_rooms: {knowledge_functions.location_rooms}")
        print(f"Location_categories: {knowledge_functions.location_categories}")
        print(f"Location_names: {knowledge_functions.location_names}")
        print(f"Object_names: {knowledge_functions.object_names}")
        print(f"Object_categories: {knowledge_functions.object_categories}")
        print(f"Object_colors: {knowledge_functions.object_colors}")
        print(f"Object_sizes: {knowledge_functions.object_sizes}")
        print(f"Object_weights: {knowledge_functions.object_weights}")

        self.assertEqual(knowledge_functions.get_location_from_room("kitchen"), "salon_table")
        print(f"Object names of category: {knowledge_functions.object_names_of_category('food')}")

        print(f"Drinks: {knowledge_functions.drink_names}")
        print(f"Drink spec: {knowledge_functions.drink_spec}")

        self.assertTrue(knowledge_functions.is_location("dinner_table"))
        self.assertEqual(knowledge_functions.get_room("dinner_table"), "livingroom")
        self.assertTrue(knowledge_functions.is_room("livingroom"))
        self.assertEqual(len(knowledge_functions.get_inspect_areas("closet")), 5)
        salon_table_inspect_areas = knowledge_functions.get_inspect_areas("salon_table")
        self.assertEqual(len(salon_table_inspect_areas), 1)
        self.assertEqual(salon_table_inspect_areas[0], "on_top_of")
        self.assertEqual(knowledge_functions.get_inspect_position("salon_table"), "in_front_of")
        self.assertTrue(knowledge_functions.is_pick_location("dinner_table"))
        self.assertFalse(knowledge_functions.is_pick_location("fridge"))
        self.assertTrue(knowledge_functions.is_place_location("dinner_table"))
        self.assertFalse(knowledge_functions.is_place_location("fridge"))
        self.assertTrue(knowledge_functions.is_known_object("mango"))
        self.assertFalse(knowledge_functions.is_known_object("foo"))
        container_objects = knowledge_functions.get_objects("container")
        self.assertEqual(len(container_objects), 4)
        self.assertIn("cup", container_objects)
        self.assertEqual(len(knowledge_functions.get_objects("foo")), 0)
        self.assertEqual(knowledge_functions.get_object_category("cup"), "container")
        self.assertEqual(knowledge_functions.get_object_category("foo"), None)
        self.assertEqual(knowledge_functions.get_object_color("banana"), "yellow")
        self.assertEqual(knowledge_functions.get_object_color("foo"), None)
        self.assertEqual(knowledge_functions.get_object_size("banana"), 320)
        self.assertEqual(knowledge_functions.get_object_size("foo"), None)
        self.assertEqual(knowledge_functions.get_object_weight("banana"), 100)
        self.assertEqual(knowledge_functions.get_object_weight("foo"), None)
        location, area_name = knowledge_functions.get_object_category_location("container")
        self.assertEqual(location, "cabinet")
        self.assertEqual(area_name, "on_top_of")




if __name__ == "__main__":
    unittest.main()
