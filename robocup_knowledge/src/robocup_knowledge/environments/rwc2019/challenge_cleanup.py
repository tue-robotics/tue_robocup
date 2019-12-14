# CLEAN UP KNOWLEDGE FILE ROBOTICS_TESTLABS
from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

"""
Local knowledge info needed:
The room to search has to have enough (way)points to cover the complete area.
This means waypoints on the floor, and known (furniture) object points.
Exact coordinates of the locations are in ed_object_models.
"""

# TODO: check
starting_point = "initial_pose"
waiting_point = "cleanup_initial"  # ToDo; CHANGE

cleaning_locations = [
    {'name': 'bedroom_chest',   'room': 'bedroom',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'bed',             'room': 'bedroom',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'sidetable',       'room': 'bedroom',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'shelf',           'room': 'bedroom',      'navigation_area': 'in_front_of', 'segment_areas': ['shelf2','shelf3','shelf4','shelf5']},

#    {'name': 'trash_bin',       'room': 'kitchen',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'kitchen_cabinet', 'room': 'kitchen',      'navigation_area': 'in_front_of', 'segment_areas': ['shelf1','shelf2','shelf3']},
    {'name': 'kitchen_table',   'room': 'kitchen',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'island',          'room': 'kitchen',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'sink',            'room': 'kitchen',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'dishwasher',      'room': 'kitchen',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'fridge',          'room': 'kitchen',      'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},

    {'name': 'shoe_rack',       'room': 'office',       'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'safe',            'room': 'office',       'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'desk',            'room': 'office',       'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
 #   {'name': 'coat_hanger',     'room': 'office',       'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},

    {'name': 'coffee_table',    'room': 'living_room',  'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'couch',           'room': 'living_room',  'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'armchair',        'room': 'living_room',  'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'display_cabinet', 'room': 'living_room',  'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
#    {'name': 'trash_bin1',      'room': 'living_room',  'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']},
    {'name': 'sideboard',       'room': 'living_room',  'navigation_area': 'in_front_of', 'segment_areas': ['on_top_of']}
]

trashbin_id = "trash_bin"

grammar_target = "T"

grammar = ""
for room in common.location_rooms:
    grammar += "\nT[{0}] -> {0}".format(room)

category_grammar = """
T[P] -> CATEGORY[P] | it is a CATEGORY[P] | the category is CATEGORY[P]
"""
for l in common.category_locations:
    category_grammar += "\nCATEGORY[{}] -> {}".format(l, l.replace('_', ' '))

category_grammar += "\nCATEGORY[{}] -> {}".format("trash", "trash".replace('_', ' '))


if __name__ == "__main__":
    print("Clean-up Grammar:\n\n{}\n\n".format(category_grammar))

    from grammar_parser.cfgparser import CFGParser

    import sys
    grammar_parser = CFGParser.fromstring(category_grammar)

    if len(sys.argv) > 2:
        sentence = " ".join(sys.argv[2:])
    else:
        sentence = grammar_parser.get_random_sentence("T")

    print("Parsing sentence:\n\n{}\n\n".format(sentence))

    result = grammar_parser.parse("T", sentence)

    print("Result:\n\n{}".format(result))
