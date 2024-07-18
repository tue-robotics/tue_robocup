# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# General
starting_point = 'initial_pose_stickler_for_the_rules'
waypoint_ids = ["hallway", "living_room", "kitchen", "office"]

# Detection
forbidden_room = "office"
forbidden_room_waypoint = "forbidden_room_waypoint"

drinks_entity_id = "kitchen_cabinet"
