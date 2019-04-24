from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

"""
Local knowledge info needed:
The room to search has to have enough (way)points to cover the complete area.
This means waypoints on the floor, and known (furniture) object points.
Exact coordinates of the locations are in ed_object_models.
"""

starting_point = "initial_pose"

# required keys: entity_id (str), room_id (str), navigation_area (str), segment_areas (list)
inspection_places = [
    {"entity_id": "dinner_table",
     "room_id": "livingroom",
     "navigate_area": "near",
     "segment_areas": ["on_top_of"]},

    {"entity_id": "cabinet",
     "room_id": "kitchen",
     "navigate_area": "near",
     "segment_areas": ["on_top_of"]},
]

rooms = [("livingroom", 2), ("kitchen", 2)]

waypoint_properties = {#"entity_id": "cleanup_wp1",
                       #"room_id":   "livingroom",
                       "navigate_area": "near",
                       "segment_areas": ["on_top_of"]}

for room, wp_count in rooms:
    for i in range(1, wp_count + 1):
        place = waypoint_properties.copy()
        place['entity_id'] = "cleanup_wp_{}{}".format(room, i)
        place['room_id'] = room
        inspection_places.append(place)

