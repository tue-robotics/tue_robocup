# CLEAN UP KNOWLEDGE FILE ROBOTICS_TESTLABS
from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

"""
Local knowledge info needed:
The room to search has to have enough (way)points to cover the complete area.
This means waypoints on the floor, and known (furniture) object points.
Exact coordinates of the locations are in ed_object_models.
"""

starting_point = "initial_pose"
waiting_point = "gpsr_meeting_point"

# required keys: entity_id (str), room_id (str), navigation_area (str), segment_areas (list)
cleaning_locations = [
    {'name': 'couch_table',   'room': 'livingroom', 'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'dinner_table',  'room': 'livingroom', 'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'cabinet',       'room': 'kitchen',    'navigate_area': 'near',   'segment_areas': ['on_top_of']},
    {'name': 'hallway_table', 'room': 'hallway',    'navigate_area': 'near',   'segment_areas': ['on_top_of']}
]

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
