from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

starting_point = "initial_pose"
ask_waypoint = "gpsr_meeting_point"

translations = {"bookcase": "bocase"}


# required keys: entity_id (str), room_id (str), navigation_area (str), segment_areas (list)
inspection_places = [
    {"entity_id": "dinner_table",
     "room_id": "livingroom",
     "navigate_area": "in_front_of",
     "segment_areas": ["on_top_of"]},

    {"entity_id": "hallway_table",
     "room_id": "hallway",
     "navigate_area": "in_front_of",
     "segment_areas": ["on_top_of"]},

    {"entity_id": "cabinet",
     "room_id": "kitchen",
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
