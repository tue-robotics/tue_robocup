# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# General
starting_point = 'initial_pose_stickler_for_the_rules'
waypoint_ids = ["office", "hallway", "living_room", "kitchen"]

# Detection
forbidden_room = "office"
forbidden_room_waypoint = "forbidden_room_waypoint"

drinks_entity_id, drinks_entity_volume = common.get_object_category_location("drinks")
