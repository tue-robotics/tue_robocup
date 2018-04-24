starting_point = "initial_pose"

ask_waypoint = "challenge_open_ask_waypoint"

# required keys: entity_id (str), room_id (str), navigation_area (str), segment_areas (list)
inspection_places = [
    {"entity_id": "dinner_table",
     "room_id": "dining_room",
     "navigate_area": "near",
     "segment_areas": ["on_top_of"]},

    {"entity_id": "desk",
     "room_id": "kitchen",
     "navigate_area": "near",
     "segment_areas": ["on_top_of"]},

    {"entity_id": "kitchencounter",
     "room_id": "living_room",
     "navigate_area": "in_front_of",
     "segment_areas": ["on_top_of"]},

    {"entity_id": "stove",
     "room_id": "kitchen",
     "navigate_area": "in_front_of",
     "segment_areas": ["on_top_of"]},
]

grammar = "T[X] -> PLACES[X]"
grammar_target = "T"
for place in inspection_places:
    grammar += "\nPLACES[%s] -> %s" % (place['entity_id'], place['entity_id'])
    grammar += "\nPLACES[%s] -> %s" % (place['entity_id'], place['room_id'])
