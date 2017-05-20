from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

# initial_pose = "initial_pose_door_B"  # initial pose
# exit_waypoint = "exit_door_B1"
# starting_point = "gpsr_meeting_point"  # Designated pose to wait for commands
starting_point = "initial_pose"
ask_waypoint = "gpsr_meeting_point"

translations = {"bookcase": "bocase"}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# We have no rooms, because we're mapping:
#
# rooms = []
# common.rooms = []
#
# def get_room(location):
#     return None

# required keys: entity_id (str), room_id (str), navigation_area (str), segment_areas (list)
inspection_places = [
    {"entity_id": "dinner_table",
     "room_id": "livingroom",
     "navigate_area": "near",
     "segment_areas": ["on_top_of"]},

    {"entity_id": "battery_table",
     "room_id": "workshop",
     "navigate_area": "in_front_of",
     "segment_areas": ["on_top_of"]},

    {"entity_id": "cabinet",
     "room_id": "hallway",
     "navigate_area": "in_front_of",
     "segment_areas": ["on_top_of"]},

    {"entity_id": "bookcase",
     "room_id": "livingroom",
     "navigate_area": "in_front_of",
     "segment_areas": ["shelf2", "shelf3", "shelf4", "shelf5"]},
]

grammar = "T[X] -> PLACES[X]"
grammar_target = "T"
for place in inspection_places:
    grammar += "\nPLACES[%s] -> %s" % (place['entity_id'], place['entity_id'])
    grammar += "\nPLACES[%s] -> %s" % (place['entity_id'], place['room_id'])
