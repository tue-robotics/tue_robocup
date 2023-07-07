# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# General
starting_point = 'initial_pose'
waypoint_ids = ["living_room", "kitchen", "bedroom", "study"]

# Detection
forbidden_room = "living_room"
forbidden_room_waypoint = "forbidden_room_waypoint"
