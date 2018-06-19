from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

not_understood_sentences = [
        "I'm so sorry! Can you please speak louder and slower? And wait for the ping!",
        "I am deeply sorry. Please try again, but wait for the ping!",
        "You and I have communication issues. Speak up!",
        "All this noise is messing with my audio. Try again"
    ]

initial_pose = "initial_pose"
starting_pose = "gpsr_meeting_point"
exit_waypoint = "gpsr_exit_door"

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
C[{"actions": <A1, A2, A3>}] -> VP[A1] VP[A2] and VP[A3]
"""

##############################################################################
#
# Verbs & shared stuff
#
##############################################################################

grammar += """
V_GUIDE -> guide | escort | take | lead | accompany | conduct

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

for name in common.names:
    grammar += "\nNAMED_PERSON[{'type': 'person', 'id': '%s'}] -> %s" % (name, name)
    for loc in common.get_locations():
        grammar += "\nPERSON_AT_LOCATION[{'type': 'person', 'id': '%s', 'location': {'id': %s}}] -> %s at the %s" % (name, loc, name, loc)

grammar += '\nLOCATION[{"id": "gpsr_exit_door_1", "type": "waypoint"}] -> exit'
grammar += '\nLOCATION[{"id": "initial_pose", "type": "waypoint"}] -> entrance'

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
# Inspect
#
###############################################################################

grammar += """
V_INSPECT -> inspect

VP[{"action": "inspect", "object": X}] -> V_INSPECT the LOCATION[X]
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

VP[{"action": "place", "object": X, "target-location": Y}] -> V_PLACE DET NAMED_OBJECT[X] MANIPULATION_AREA_LOCATION[Y]
VP[{"action": "place", "target-location": X, "object": {"type": "reference"}}] -> V_PLACE PPN_OBJECT MANIPULATION_PP the LOCATION[X]
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

V_BRING -> bring | deliver | take | give | V_PICKUP
"""

# HAND OVER

grammar += """
VP[{"action": "hand-over", "target-location": Y, "object": Z}] -> V_BRING OPERATOR[Y] DET NAMED_OBJECT[Z]
VP[{"action": "hand-over", "source-location": X, "target-location": Y, "object": Z}] -> V_BRING OPERATOR[Y] DET NAMED_OBJECT[Z] from the LOCATION[X]
VP[{"action": "hand-over", "source-location": X, "target-location": Y, "object": Z}] -> V_BRING to PERSON_AT_LOCATION[Y] DET NAMED_OBJECT[Z] from the LOCATION[X]

VP[{"action": "hand-over", "target-location": Y, "object": Z}] -> V_BRING DET NAMED_OBJECT[Z] to PERSON_AT_LOCATION[Y]
VP[{"action": "hand-over", "target-location": Y, "object": Z}] -> V_BRING DET NAMED_OBJECT[Z] to OPERATOR[Y]


VP[{"action": "hand-over", "target-location": X, "object": {"type": "reference"}}] -> V_BRING PPN_OBJECT to PERSON_AT_LOCATION[X]
VP[{"action": "hand-over", "target-location": X, "object": {"type": "reference"}}] -> V_BRING PPN_OBJECT to OPERATOR[X]
"""

# PLACE

grammar += """
VP[{"action": "place", "target-location": Y, "object": Z}] -> V_BRING DET NAMED_OBJECT[Z] to the LOCATION[Y]
VP[{"action": "place", "source-location": X, "target-location": Y, "object": Z}] -> V_BRING DET NAMED_OBJECT[Z] from the LOCATION[X] to the LOCATION[Y]
"""

##############################################################################
#
# SAY
#
##############################################################################

grammar += """
V_SAY -> tell | say

VP[{"action": "say", "sentence": X}] -> V_SAY SAY_SENTENCE[X]
VP[{"action": "say", "sentence": X, "object": Y}] -> V_SAY SAY_SENTENCE[X] to PERSON_AT_LOCATION[Y]
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

##############################################################################
#
# ANSWER QUESTION
#
##############################################################################

grammar += """
V_ANSWER_QUESTION -> answer a question
VP[{"action": "answer-question"}] -> V_ANSWER_QUESTION
VP[{"action": "answer-question", "target-person": X}] -> V_ANSWER_QUESTION to NAMED_PERSON[X]
VP[{"action": "answer-question", "target-person": X}] -> V_ANSWER_QUESTION to PERSON_AT_LOCATION[X]
"""

##############################################################################
#
# FIND OUT AND REPORT
#
##############################################################################

grammar += """
VP[{"action": "tell-name-of-person", "location": X}] -> tell me the name of the person MEETING_PP the ROOM_OR_LOCATION[X]
VP[{"action": "count-and-tell", "object": X, "location": Y}] -> tell me how many NAMED_OBJECT[X] there are on the LOCATION[Y]
"""


if __name__ == "__main__":
    print "GPSR Grammar:\n\n{}\n\n".format(grammar)

    from grammar_parser.cfgparser import CFGParser

    import sys
    grammar_parser = CFGParser.fromstring(grammar)

    if len(sys.argv) > 2:
        sentence = " ".join(sys.argv[2:])
    else:
        sentence = grammar_parser.get_random_sentence("T")

    print "Parsing sentence:\n\n{}\n\n".format(sentence)

    result = grammar_parser.parse("T", sentence)

    print "Result:\n\n{}".format(result)
