# CLEAN UP KNOWLEDGE FILE RGO2019

from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

"""
Local knowledge info needed:
The room to search has to have enough (way)points to cover the complete area.
This means waypoints on the floor, and known (furniture) object points.
Exact coordinates of the locations are in ed_object_models.
"""

starting_point = "initial_pose"
starting_pose = "gpsr_meeting_point"

# required keys: entity_id (str), room_id (str), navigation_area (str), segment_areas (list)

cleaning_locations = [
    {'name': 'sideboard',       'room': 'living_room', 'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'tv_table',        'room': 'living_room', 'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'left_arm_chair',  'room': 'living_room', 'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'coffee_table',    'room': 'living_room', 'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'couch',           'room': 'living_room', 'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'right_arm_chair', 'room': 'living_room', 'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'high_table',      'room': 'living_room', 'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'bookcase',        'room': 'living_room', 'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'trash_bin',       'room': 'living_room', 'navigate_area': 'near',   'segment_areas': ['on_top_of']},

    {'name': 'desk',            'room': 'bedroom',     'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'bed',             'room': 'bedroom',     'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'side_table',      'room': 'bedroom',     'navigate_area': 'near',   'segment_areas': ['on_top_of']},

    {'name': 'kitchen_cabinet', 'room': 'kitchen',     'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'dishwasher',      'room': 'kitchen',     'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'kitchen_table',   'room': 'kitchen',     'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'cabinet',         'room': 'kitchen',     'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'trash_can',       'room': 'kitchen',     'navigate_area': 'near',   'segment_areas': ['on_top_of']},

    {'name': 'bar_table',       'room': 'bar',         'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'sofa',            'room': 'bar',         'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'cupboard',        'room': 'bar',         'navigate_area': 'near',   'segment_areas': ['on_top_of']}
]
grammar_target = "T"

grammar = "T -> kitchen"
grammar += "\nT -> livingroom"
