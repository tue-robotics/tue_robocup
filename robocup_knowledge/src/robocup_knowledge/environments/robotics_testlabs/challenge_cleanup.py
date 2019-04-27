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
cleaning_locations = [
    {'name': 'couch_table',   'room': 'livingroom', 'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'dinner_table',  'room': 'livingroom', 'navigate_area': 'near',   'segment_areas': ['on_top_of']},

    {'name': 'cabinet',       'room': 'kitchen',    'navigate_area': 'near',   'segment_areas': ['on_top_of']},

    {'name': 'hallway_table', 'room': 'hallway',    'navigate_area': 'near',   'segment_areas': ['on_top_of']}
]
