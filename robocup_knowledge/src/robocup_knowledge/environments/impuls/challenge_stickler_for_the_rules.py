# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# General
starting_point = 'initial_pose'
waypoint_ids = ["living_room", "kitchen", "dining_room", "office"]

# Detection
forbidden_room = "office"
forbidden_room_waypoint = "office"

drinks_entity_id = "fridge"
