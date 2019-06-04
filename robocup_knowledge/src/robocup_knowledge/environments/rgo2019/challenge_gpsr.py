from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")
spr = knowledge_loader.load_knowledge("challenge_spr")

not_understood_sentences = [
        "I'm so sorry! Can you please speak clearly and fluently? And wait for the ping!",
        "I am deeply sorry. Please try again, but wait for the ping!",
        "You and I have communication issues. Speak clearly!",
        "All this noise is messing with my audio. Try again."
    ]

initial_pose = "gpsr_meeting_point"
starting_pose = "gpsr_meeting_point"
exit_waypoint = "exit_1_rips"

grammar_target = "T"

##############################################################################
#
# Actions
#
##############################################################################

grammar = """
T[A] -> COURTESY_PREFIX C[A] | C[A]

C[{"actions": <A1>}] -> VP[A1]
C[{"actions": <A1, A2>}] -> VP[A1] and VPT[A2]
C[{"actions": <A1, A2, A3>}] -> VP[A1] VP[A2] and VPT[A3]

VPT -> VP | VPS

COURTESY_PREFIX -> robot please | could you | could you please | please
"""

##############################################################################
#
# Verbs & shared stuff
#
##############################################################################

grammar += """
PPN_OBJECT -> it
PPN_PERSON -> him | her

OPERATOR_PROPERTY -> my
GEN_VERB -> may | can | will

DET -> the | a
NUMBER -> one | two | three
MEETING_PP -> at | in
MANIPULATION_PP -> on | to
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
    grammar += "\nPERSON_AT_LOCATION[{'type': 'person', 'id': '%s', 'location': {'id': 'gpsr_entrance', 'type': 'waypoint'}}] -> %s at the entrance" % (
    name, name)
    for loc in common.get_locations():
        grammar += "\nPERSON_AT_LOCATION[{'type': 'person', 'id': '%s', 'location': {'id': %s}}] -> %s at the %s" % (name, loc, name, loc)

grammar += '\nLOCATION[{"id": "gpsr_entrance", "type": "waypoint"}] -> entrance'


###############################################################################
#
# FIND
#
###############################################################################

grammar += """
V_FIND -> find | locate | look for | pinpoint | spot | meet

TYPE_OR_CATEGORY -> NAMED_OBJECT | OBJECT_CATEGORY
UNNAMED_PERSON -> a person | someone

VP[{"action": "find", "object": X}] -> V_FIND NAMED_PERSON[X]
VP[{"action": "find", "object": X, "source-location": Y}] -> V_FIND NAMED_PERSON[X] MEETING_PP the ROOM_OR_LOCATION[Y]
VP[{"action": "find", "object": X, "source-location": Y}] -> V_FIND DET TYPE_OR_CATEGORY[X] in the ROOM[Y]
VP[{"action": "find"}] -> V_FIND DET NAMED_OBJECT
"""

## Unused actions in rgo2019
# VP[{"action": "find", "object": {'type': 'person'}}] -> V_FIND UNNAMED_PERSON
# VP[{"action": "find", "source-location": Y}] -> V_FIND DET object in the ROOM[Y]
# VP[{"action": "find", "object": X}] -> V_FIND DET TYPE_OR_CATEGORY[X]

# ##############################################################################
#
# Follow
#
# ##############################################################################

grammar += """
V_FOLLOW -> come behind | come after | follow | go after | go behind

VP[{"action": "follow", "target": X}] -> V_FOLLOW NAMED_PERSON[X]
VP[{"action": "follow", "target": X, "source-location": Y, "target-location": Z}] -> V_FOLLOW NAMED_PERSON[X] from DET ROOM_OR_LOCATION[Y] to DET ROOM_OR_LOCATION[Z]
VP[{"action": "follow", "target-location": Y, "target": {"type": "reference"}}] -> V_FOLLOW PPN_PERSON to DET ROOM_OR_LOCATION[Y]
VP[{"action": "follow", "target": {"type": "reference"}}] -> V_FOLLOW PPN_PERSON
"""
## Unused actions in rgo2019
# VP[{"action": "follow", "object": X, "location-from":Y, "location-to":Z}] -> V_FOLLOW NAMED_PERSON[X] from DET LOCATION[Y] to DET LOCATION[Z]

###############################################################################
#
# Guide
#
###############################################################################

grammar += """
V_GUIDE -> guide | escort | take | lead | accompany | conduct

VP[{"action": "guide", "object": X}] -> V_GUIDE NAMED_PERSON[X]
VP[{"action": "guide", "object": X, "target-location": Y}] -> V_GUIDE NAMED_PERSON[X] to the LOCATION[Y]
VP[{"action": "guide", "object": X, "source-location": Y, "target-location": Z}] -> V_GUIDE NAMED_PERSON[X] to the LOCATION[Z] you GEN_VERB find PPN_PERSON MEETING_PP the LOCATION[Y]
VP[{"action": "guide", "object": X, "source-location": Y, "target-location": Z}] -> V_GUIDE NAMED_PERSON[X] from the LOCATION[Y] to the LOCATION[Z]
VPS[{"action": "guide", "object": {"type": "reference"}}] -> V_GUIDE PPN_PERSON
VP[{"action": "guide", "target-location": Y, "object": {"type": "reference"}}] -> V_GUIDE PPN_PERSON to the ROOM_OR_LOCATION[Y]
"""


###############################################################################
#
# Navigate
#
###############################################################################

grammar += """
V_GOTO -> go to | navigate to | enter to

VP[{"action": "navigate-to", "target-location": X}] -> V_GOTO the ROOM_OR_LOCATION[X]
"""


###############################################################################
#
# Pick-up
#
###############################################################################

grammar += """
V_PICKUP -> get | grasp | take | pick up | grab | retrieve

VP[{"action": "pick-up", "object": {"type": "reference"}, "source-location": Y}] -> V_PICKUP PPN_OBJECT from the LOCATION[Y]
VP[{"action": "pick-up", "object": X, "source-location": Y}] -> V_PICKUP DET TYPE_OR_CATEGORY[X] from the LOCATION[Y]
VP[{"action": "pick-up", "object": X}] -> V_PICKUP DET NAMED_OBJECT[X]
"""
## Unused in rgo2019
# VP[{"action": "pick-up", "object": X, "source-location": Y}] -> V_PICKUP DET TYPE_OR_CATEGORY[X] from the LOCATION[Y]

###############################################################################
#
# Hand over
#
###############################################################################

grammar += """
OPERATOR[{"type": "person", "id": "operator"}] -> me
BRING_NAME -> OPERATOR | NAMED_PERSON

OBJECT_TO_BE_BROUGHT -> NAMED_OBJECT | DET NAMED_OBJECT

V_BRING -> bring | deliver | give | hand over | hand | take
"""

grammar += """

VP[{"action": "hand-over", "target-location": Y, "category": Z}] -> V_BRING OBJECT_CATEGORY[Z] to OPERATOR[Y]  
VP[{"action": "hand-over", "target-location": Y, "category": Z}] -> V_BRING OPERATOR[Y] DET OBJECT_CATEGORY[Z]    


VP[{"action": "hand-over", "target-location": Y, "object": Z}] -> V_BRING OPERATOR[Y] DET NAMED_OBJECT[Z]
VP[{"action": "hand-over", "target-location": Y, "object": Z}] -> V_BRING OPERATOR_PROPERTY NAMED_OBJECT[Z] to the LOCATION[Y]
VP[{"action": "hand-over", "target-location": Y, "object": Z}] -> V_BRING DET NAMED_OBJECT[Z] to the LOCATION[Y]
VP[{"action": "hand-over", "target-location": Y, "object": Z}] -> V_BRING DET NAMED_OBJECT[Z] to OPERATOR[Y]
VPS[{"action": "hand-over", "target-location": X, "object": {"type": "reference"}}] -> V_BRING PPN_OBJECT to PERSON_AT_LOCATION[X]
VPS[{"action": "hand-over", "target-location": X, "object": {"type": "reference"}}] -> V_BRING PPN_OBJECT to OPERATOR[X]
"""

## Unused grammar in rgo 2019
# VP[{"action": "hand-over", "source-location": X, "target-location": Y, "object": Z}] -> V_BRING DET NAMED_OBJECT[Z] from the ROOM[X] to the LOCATION[Y]
# VP[{"action": "hand-over", "source-location": X, "target-location": Y, "object": Z}] -> V_BRING OPERATOR[Y] DET NAMED_OBJECT[Z] from the LOCATION[X]
# VP[{"action": "hand-over", "source-location": X, "target-location": Y, "object": Z}] -> V_BRING to PERSON_AT_LOCATION[Y] DET NAMED_OBJECT[Z] from the LOCATION[X]
# VP[{"action": "hand-over", "target-location": Y, "object": Z}] -> V_BRING DET NAMED_OBJECT[Z] to PERSON_AT_LOCATION[Y]
# VP[{"action": "hand-over", "target-location": Y, "object_category": Z}] -> V_BRING OPERATOR[Y] DET OBJECT_CATEGORY[Z]
# VP[{"action": "hand-over", "target-location": Y, "object_category": Z}] -> V_BRING OBJECT_CATEGORY[Z] to OPERATOR[Y]

## PLACE (within handover)
#
# grammar += """
# VP[{"action": "place", "target-location": Y, "object": Z}] -> V_BRING DET NAMED_OBJECT[Z] to the LOCATION[Y]
# VP[{"action": "place", "source-location": X, "target-location": Y, "object": Z}] -> V_BRING DET NAMED_OBJECT[Z] from the LOCATION[X] to the LOCATION[Y]
# """


###############################################################################
#
# Place
#
###############################################################################

grammar += """
V_PLACE -> put | place | leave | set

VP[{"action": "place", "target-location": X, "object": {"type": "reference"}}] -> V_PLACE PPN_OBJECT on the LOCATION[X]
"""
## Unused actions in rgo2019
# VP[{"action": "place", "object": X, "target-location": Y}] -> V_PLACE NAMED_OBJECT[X] MANIPULATION_AREA_LOCATION[Y]
# VP[{"action": "place", "object": X, "target-location": Y}] -> V_PLACE DET NAMED_OBJECT[X] MANIPULATION_AREA_LOCATION[Y]
# VP[{"action": "place", "object": {"type": "reference"}}] -> V_PLACE PPN_OBJECT to the ROOM

##############################################################################
#
# Say
#
##############################################################################

grammar += """
V_SAY -> tell | say | ask
V_SAY_UNDEFINED -> speak | say something

VP[{"action": "say", "sentence": X}] -> V_SAY SAY_SENTENCE[X]
"""
grammar += '\nSAY_SENTENCE["can you please leave"] -> PPN_PERSON to leave'
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

## Unused in rgo2019
# VP[{"action": "say"}] -> V_SAY_UNDEFINED

##############################################################################
#
# ANSWER QUESTION
#
##############################################################################

grammar += """
V_ANSWER_QUESTION -> answer a question
VP[{"action": "answer-question"}] -> V_ANSWER_QUESTION
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
