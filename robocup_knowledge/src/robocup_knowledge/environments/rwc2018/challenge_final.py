from __future__ import print_function

from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

grammar_target = "T"

##############################################################################
#
# Actions
#
##############################################################################

grammar = """
T[A] -> C[A]

C[{"actions": <A1>}] -> VP[A1]
C[{"actions": <A1, A2>}] -> VP[A1] and VP[A2]
"""

##############################################################################
#
# Names for this specific challenge
#
##############################################################################
female_names = ["josja"]
male_names = ["max", "lars", "rein", "rokus", "ramon", "loy", "sam", "henk", "matthijs", "lieve"]
names = female_names + male_names

##############################################################################
#
# Verbs & shared stuff
#
##############################################################################

grammar += """
PPN_OBJECT -> it | them
PPN_PERSON -> him | her | them

DET -> the
NUMBER -> one | two | three
MEETING_PP -> at | in
MANIPULATION_PP -> on
"""

for room in common.location_rooms:
    grammar += "\nROOM[{'type': 'room', 'id': '%s'}] -> %s" % (room, room)

for loc in common.get_locations():
    grammar += '\nLOCATION[{"id": "%s"}] -> %s' % (loc, loc)

grammar += '\n ROOM_OR_LOCATION[X] -> ROOM[X] | LOCATION[X]'

for obj in common.object_names:
    grammar += "\nNAMED_OBJECT[{'type': '%s'}] -> %s" % (obj, obj)

for loc in common.get_locations(pick_location=True, place_location=True):
    grammar += '\nMANIPULATION_AREA_LOCATION[{"id": "%s"}] -> MANIPULATION_PP the %s' % (loc, loc)

for cat in common.object_categories:
    grammar += "\nOBJECT_CATEGORY[{'category': '%s'}] -> %s" % (cat, cat)

for name in names:
    grammar += "\nNAMED_PERSON[{'type': 'person', 'id': '%s'}] -> %s" % (name, name)
    grammar += "\nPERSON_AT_LOCATION[{'type': 'person', 'id': '%s', 'location': {'id': 'gpsr_entrance', 'type': 'waypoint'}}] -> %s at the entrance" % (
        name, name)
    grammar += "\nPERSON_AT_LOCATION[{'type': 'person', 'id': '%s', 'location': {'id': 'gpsr_exit_door', 'type': 'waypoint'}}] -> %s at the exit" % (
        name, name)
    for loc in common.get_locations():
        grammar += "\nPERSON_AT_LOCATION[{'type': 'person', 'id': '%s', 'location': {'id': %s}}] -> %s at the %s" % (
        name, loc, name, loc)

grammar += '\nLOCATION[{"id": "gpsr_exit_door", "type": "waypoint"}] -> exit'
grammar += '\nLOCATION[{"id": "gpsr_entrance", "type": "waypoint"}] -> entrance'

###############################################################################
#
# FIND
#
###############################################################################

grammar += """
V_FIND -> find | locate | look for

OBJECT_TO_BE_FOUND -> NAMED_OBJECT | OBJECT_CATEGORY
UNNAMED_PERSON -> a person | someone

VP[{"action": "find", "object": {'type': 'person'}}] -> V_FIND UNNAMED_PERSON
VP[{"action": "find", "object": {'type': 'person'}, "source-location": Y}] -> V_FIND UNNAMED_PERSON in the ROOM[Y]
VP[{"action": "find", "object": X}] -> V_FIND NAMED_PERSON[X]
VP[{"action": "find", "object": X, "source-location": Y}] -> V_FIND NAMED_PERSON[X] in the ROOM[Y]

VP[{"action": "find", "object": X, "source-location": Y}] -> V_FIND DET OBJECT_TO_BE_FOUND[X] in the ROOM[Y]

VP[{"action": "find", "object": X}] -> V_FIND DET OBJECT_TO_BE_FOUND[X]
"""



###############################################################################
#
# Navigate
#
###############################################################################

grammar += """
V_GOTO -> go to | navigate to

VP[{"action": "navigate-to", "target-location": X}] -> V_GOTO the ROOM_OR_LOCATION[X]
"""

###############################################################################
#
# Check who's at the door
#
###############################################################################

grammar += """
V_SEND_PICTURE -> check who is | show me who is

VP[{"action": "send-picture", "target-location": X}] -> V_SEND_PICTURE at the ROOM_OR_LOCATION[X]
"""


###############################################################################
#
# Guide
#
###############################################################################

grammar += """
V_GUIDE -> guide

VP[{"action": "guide-final-challenge", "object": {"type": "reference"}, "target-location": X}] -> guide him to the ROOM_OR_LOCATION[X]
"""

###############################################################################
#
# Pick-up
#
###############################################################################

grammar += """
V_PICKUP -> get | grasp | take | pick up | grab

VP[{"action": "pick-up", "object": X, "source-location": Y}] -> V_PICKUP DET NAMED_OBJECT[X] from the LOCATION[Y]
"""

###############################################################################
#
# Place
#
###############################################################################

grammar += """
V_PLACE -> put | place

VP[{"action": "put", "object": X, "target-location": Y}] -> V_PLACE DET NAMED_OBJECT[X] on the LOCATION[Y]
"""

###############################################################################
#
# Gripper goal
#
###############################################################################

grammar += """
GOAL[open] -> open
GOAL[close] -> close

SIDE[left] -> left
SIDE[right] -> right
VP[{"action": "gripper-goal", "side": X, "goal": Y}] -> GOAL[Y] your SIDE[X] gripper
"""


###############################################################################
#
# Dishwasher
#
###############################################################################

grammar += """
VP[{"action": "clear-table"}] -> clear the dining_table
"""


###############################################################################
#
# Open cupboard door
#
###############################################################################

grammar += """
VP[{"action": "open-door", "door-location": "cupboard"}] -> open the door of the cupboard
"""


###############################################################################
#
# BRING
#
###############################################################################

grammar += """
OPERATOR[{"type": "person", "id": "operator"}] -> me
BRING_NAME -> OPERATOR | NAMED_PERSON

OBJECT_TO_BE_BROUGHT -> NAMED_OBJECT | DET NAMED_OBJECT

V_BRING -> bring
"""

###############################################################################
#
# Hand over
#
###############################################################################

grammar += """
VP[{"action": "hand-over", "target-location": Y, "object": Z}] -> V_BRING OPERATOR[Y] DET NAMED_OBJECT[Z]
VP[{"action": "hand-over", "source-location": X, "target-location": Y, "object": Z}] -> V_BRING OPERATOR[Y] DET NAMED_OBJECT[Z] from the LOCATION[X]
VP[{"action": "hand-over", "source-location": X, "target-location": Y, "object": Z}] -> V_BRING to PERSON_AT_LOCATION[Y] DET NAMED_OBJECT[Z] from the LOCATION[X]

VP[{"action": "hand-over", "target-location": Y, "object": Z}] -> V_BRING DET NAMED_OBJECT[Z] to PERSON_AT_LOCATION[Y]
VP[{"action": "hand-over", "target-location": Y, "object": Z}] -> V_BRING DET NAMED_OBJECT[Z] to OPERATOR[Y]


VP[{"action": "hand-over", "target-location": X, "object": {"type": "reference"}}] -> V_BRING PPN_OBJECT to PERSON_AT_LOCATION[X]
VP[{"action": "hand-over", "target-location": X, "object": {"type": "reference"}}] -> V_BRING PPN_OBJECT to OPERATOR[X]
"""

##############################################################################
#
# SAY
#
##############################################################################

grammar += """
V_SAY -> tell | say

VP[{"action": "say", "sentence": X}] -> V_SAY SAY_SENTENCE[X]
VP[{"action": "say", "sentence": "party"}] -> lets get the party started
"""

grammar += '\nSAY_SENTENCE["time"] -> the time'
grammar += '\nSAY_SENTENCE["team_name"] -> your teams name'
grammar += '\nSAY_SENTENCE["country"] -> your teams country'
grammar += '\nSAY_SENTENCE["team_affiliation"] -> your teams affiliation'
grammar += '\nSAY_SENTENCE["day_of_month"] -> the day of the month'
grammar += '\nSAY_SENTENCE["day_of_week"] -> the day of the week'
grammar += '\nSAY_SENTENCE["today"] -> what day is today'
grammar += '\nSAY_SENTENCE["tomorrow"] -> what day is tomorrow'
grammar += '\nSAY_SENTENCE["joke"] -> a joke'
grammar += '\nSAY_SENTENCE["something_about_self"] -> something about yourself'
grammar += '\nSAY_SENTENCE["you_shall_not_pass"] -> you shall not pass'


if __name__ == "__main__":
    print("GPSR Grammar:\n\n{}\n\n".format(grammar))

    from grammar_parser.cfgparser import CFGParser

    import sys
    grammar_parser = CFGParser.fromstring(grammar)

    if len(sys.argv) > 2:
        sentence = " ".join(sys.argv[2:])
    else:
        sentence = grammar_parser.get_random_sentence("T")

    print("Parsing sentence:\n\n{}\n\n".format(sentence))

    result = grammar_parser.parse("T", sentence)

    print("Result:\n\n{}".format(result))
