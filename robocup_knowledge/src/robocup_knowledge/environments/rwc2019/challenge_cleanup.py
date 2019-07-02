# CLEAN UP KNOWLEDGE FILE ROBOTICS_TESTLABS
from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

"""
Local knowledge info needed:
The room to search has to have enough (way)points to cover the complete area.
This means waypoints on the floor, and known (furniture) object points.
Exact coordinates of the locations are in ed_object_models.
"""

#TODO:check
starting_point = "initial_pose"
# starting_pose = "gpsr_meeting_point"

locations = [
    {'name': 'bedroom_chest',   'room': 'bedroom',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'bed',             'room': 'bedroom',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'sidetable',      'room': 'bedroom',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'shelf',           'room': 'bedroom',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},

    {'name': 'trash_bin',       'room': 'kitchen',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'kitchen_cabinet', 'room': 'kitchen',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'kitchen_table',   'room': 'kitchen',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'island',          'room': 'kitchen',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'sink',            'room': 'kitchen',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'dishwasher',      'room': 'kitchen',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'fridge',          'room': 'kitchen',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},

    {'name': 'shoe_rack',       'room': 'office',       'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'safe',            'room': 'office',       'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'desk',            'room': 'office',       'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'coat_hanger',     'room': 'office',       'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},

    {'name': 'coffee_table',    'room': 'living_room',  'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'couch',           'room': 'living_room',  'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'armchair',        'room': 'living_room',  'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'display_cabinet', 'room': 'living_room',  'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'trash_bin1',      'room': 'living_room',  'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'sideboard',       'room': 'living_room',  'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']}
]

grammar_target = "T"

grammar = ""
for room in common.rooms:
    grammar += "\nT[{0}] -> {0}".format(room)
