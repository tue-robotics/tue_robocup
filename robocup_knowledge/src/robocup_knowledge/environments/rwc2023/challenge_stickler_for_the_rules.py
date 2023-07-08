# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# General
starting_point = 'stickler_for_rules_starting_point'
waypoint_ids = ["living_room", "kitchen", "bedroom", "study"]

# Detection
forbidden_room = "bedroom"
forbidden_room_waypoint = "forbidden_room_waypoint"
