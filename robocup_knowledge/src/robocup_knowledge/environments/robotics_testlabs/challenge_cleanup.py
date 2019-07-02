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
    {'name': 'dinner_table',  'room': 'livingroom', 'navigation_area': 'in_front_of',   'segment_areas': ['on_top_of']},
    {'name': 'couch_table',   'room': 'livingroom', 'navigation_area': 'in_front_of',   'segment_areas': ['on_top_of']},
    {'name': 'cabinet',       'room': 'kitchen',    'navigation_area': 'in_front_of',   'segment_areas': ['on_top_of']},
    {'name': 'hallway_table', 'room': 'hallway',    'navigation_area': 'in_front_of',   'segment_areas': ['on_top_of']}
]

trashbin_id = "trashbin"

grammar_target = "T"

grammar = ""
for room in common.location_rooms:
    grammar += "\nT[{0}] -> {0}".format(room)

category_grammar = """
T[P] -> LOCATION[P] | bring it to the LOCATION[P] | please bring it to LOCATION[P]
"""
for l in common.locations:
    category_grammar += "\nLOCATION[{}] -> {}".format(l["name"], l["name"].replace('_', ' '))


if __name__ == "__main__":
    print("GPSR Grammar:\n\n{}\n\n".format(grammar))

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
