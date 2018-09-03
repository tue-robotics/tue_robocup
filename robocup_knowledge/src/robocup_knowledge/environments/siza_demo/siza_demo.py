from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

not_understood_sentences = [
        "I'm so sorry! Can you please speak louder and slower? And wait for the ping!",
        "I am deeply sorry. Please try again, but wait for the ping!",
        "You and I have communication issues. Speak up!",
        "All this noise is messing with my audio. Try again"
    ]

initial_pose = "hero_home"

grammar_target = "T"

##############################################################################
#
# Actions
#
##############################################################################

grammar = """
T[A] -> C[A] | COURTESY C[A] | C[A] COURTESY | COURTESY C[A] COURTESY

COURTESY -> please | robot please | could you | would you | hero | hero please
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
PPN_OBJECT -> it | them
PPN_PERSON -> him | her | them

DET -> the | a | an | some
NUMBER -> one | two | three
MANIPULATION_AREA_DESCRIPTION -> on top of | at | in | on | from
"""

for room in common.location_rooms:
    grammar += "\nROOM[{'type': 'room', 'id': '%s'}] -> %s" % (room, room)

for loc in common.get_locations():
    grammar += '\nLOCATION[{"id": "%s"}] -> %s' % (loc, loc)

grammar += '\n ROOM_OR_LOCATION[X] -> ROOM[X] | LOCATION[X]'

for obj in common.object_names:
    grammar += "\nNAMED_OBJECT[{'type': '%s'}] -> %s" % (obj, obj)

for loc in common.get_locations(pick_location=True, place_location=True):
    grammar += '\nMANIPULATION_AREA_LOCATION[{"id": "%s"}] -> MANIPULATION_AREA_DESCRIPTION the %s' % (loc, loc)

for cat in common.object_categories:
    grammar += "\nOBJECT_CATEGORY[{'category': '%s'}] -> %s" % (cat, cat)

for name in common.names:
    grammar += "\nNAMED_PERSON[{'type': 'person', 'id': '%s'}] -> %s" % (name, name)

###############################################################################
#
# Demo
#
###############################################################################

grammar += """
VP[{"action": "demo-presentation"}] -> introduce yourself | present yourself | perform a demonstration
"""

###############################################################################
#
# Send picture attempt
#
###############################################################################
grammar += """
V_SEND_PICTURE -> check what is on | show me what is on
VP[{"action": "send-picture", "target-location": X}] -> V_SEND_PICTURE the ROOM_OR_LOCATION[X]
"""


###############################################################################
#
# Clear
#
###############################################################################

grammar += """
V_CLEAR -> clear | clean up | clean-up | empty

VP[{"action": "clear", "source-location": X, "target-location": {"id":"trashbin"}}] -> V_CLEAR the ROOM_OR_LOCATION[X]
VP[{"action": "clear", "source-location": X, "target-location": Y}] -> V_CLEAR the ROOM_OR_LOCATION[X] to the ROOM_OR_LOCATION[Y]
"""

###############################################################################
#
# Return
#
###############################################################################

grammar += """
V_RETURN -> bye | no thank you | no bye | go away

VP[{"action": "navigate-to", "target-location": {"type":"waypoint", "id":"hero_home"}}] -> V_RETURN
"""

###############################################################################
#
# FIND
#
###############################################################################

grammar += """
V_FIND -> find | locate | look for | pinpoint | spot
V_FIND_PERSON -> meet | V_FIND

OBJECT_TO_BE_FOUND -> NAMED_OBJECT | OBJECT_CATEGORY
PERSON_TO_BE_FOUND -> DET person | DET woman | DET man
UNNAMED_PERSON -> a person | someone | me

VP[{"action": "find", "object": X, "source-location": Y}] -> V_FIND DET OBJECT_TO_BE_FOUND[X] in the ROOM[Y]
VP[{"action": "find", "object": {'type': 'person'}, "source-location": Y}] -> V_FIND UNNAMED_PERSON in the ROOM[Y]
VP[{"action": "find", "object": X, "source-location": Y}] -> V_FIND NAMED_PERSON[X] in the ROOM[Y]

VP[{"action": "find", "object": {'type': 'person'}}] -> V_FIND UNNAMED_PERSON
VP[{"action": "find", "object": X}] -> V_FIND DET OBJECT_TO_BE_FOUND[X]
VP[{"action": "find", "object": X}] -> V_FIND NAMED_PERSON[X]

"""
# VP[{"action": "find", "object": X, "source-location": Y}] -> V_FIND PERSON_TO_BE_FOUND[X] near the LOCATION[Y]
# VP[{"action": "find", "object": X, "source-location": Y}] -> V_FIND DET OBJECT_TO_BE_FOUND[X] MANIPULATION_AREA_LOCATION[Y]
###############################################################################
#
# Navigate
#
###############################################################################

grammar += """
V_GOPL -> go to | navigate to
V_GOR -> V_GOPL | enter to

VP[{"action": "navigate-to", "target-location": X}] -> V_GOR the ROOM[X]
VP[{"action": "navigate-to", "target-location": X}] -> V_GOPL the LOCATION[X]
"""

###############################################################################
#
# Inspect
#
###############################################################################

grammar += """

VP[{"action": "inspect", "entity": X}] -> inspect the LOCATION[X]
"""

###############################################################################
#
# Pick-up
#
###############################################################################

grammar += """
V_PICKUP -> get | grasp | take | pick up | grab

VP[{"action": "pick-up", "object": X, "source-location": Y}] -> V_PICKUP DET NAMED_OBJECT[X] MANIPULATION_AREA_LOCATION[Y]
"""

###############################################################################
#
# Place
#
###############################################################################

grammar += """
V_PLACE -> put | place | set

VP[{"action": "place", "object": X, "target-location": Y}] -> V_PLACE DET NAMED_OBJECT[X] MANIPULATION_AREA_LOCATION[Y]
VP[{"action": "place", "object": X, "target-location": Y}] -> V_PLACE PPN_OBJECT[X] MANIPULATION_AREA_LOCATION[Y]
"""

###############################################################################
#
# Follow
#
###############################################################################

grammar += """
V_FOLLOW -> follow | go after | come after | V_GUIDE

VP[{"action": "follow", "location-from": X, "location-to": Y, "target": {"id": "operator"}}] -> V_FOLLOW me from the ROOM_OR_LOCATION[X] to the ROOM_OR_LOCATION[Y]
VP[{"action": "follow", "location-to": X, "location-from": Y, "target": {"id": "operator"}}] -> V_FOLLOW me to the ROOM_OR_LOCATION[X] from the ROOM_OR_LOCATION[Y]

VP[{"action": "follow", "target": {"id": "operator"}}] -> V_FOLLOW me
VP[{"action": "follow", "target": {"id": "operator"}, "location-to": X}] -> V_FOLLOW me to the ROOM_OR_LOCATION[X]

VP[{"action": "follow", "target": {"type": "reference"}}] -> V_FOLLOW PPN_PERSON
VP[{"action": "follow", "target": {"type": "reference"}, "location-to": X}] -> V_FOLLOW PPN_PERSON to the ROOM_OR_LOCATION[X]

VP[{"action": "follow", "location-from": X, "location-to": Y, "target": Z}] -> V_FOLLOW FOLLOW_PERSONS[Z] from the ROOM_OR_LOCATION[X] to the ROOM_OR_LOCATION[Y]
VP[{"action": "follow", "location-to": X, "location-from": Y, "target": Z}] -> V_FOLLOW FOLLOW_PERSONS[Z] to the ROOM_OR_LOCATION[X] from the ROOM_OR_LOCATION[Y]

VP[{"action": "follow", "location-from": X, "target": Z}] -> V_FOLLOW FOLLOW_PERSONS[Z] from the ROOM_OR_LOCATION[X]
VP[{"action": "follow", "location-to": X, "target": Z}] -> V_FOLLOW FOLLOW_PERSONS[Z] to the ROOM_OR_LOCATION[X]

VP[{"action": "follow", "target": Z}] -> V_FOLLOW FOLLOW_PERSONS[Z]
"""

grammar += '\nFOLLOW_PERSONS[the person] -> DET person'
grammar += '\nFOLLOW_PERSONS[the woman] -> DET woman'
grammar += '\nFOLLOW_PERSONS[the man] -> DET man'

###############################################################################
#
# BRING
#
###############################################################################

# BRING_TARGET[{"id": X, "type": person}] -> BRING_NAME[X]

grammar += """
OPERATOR[{"id": "operator"}] -> me
BRING_NAME -> OPERATOR | BRING_PERSON

BRING_TARGET[X] -> the ROOM_OR_LOCATION[X]

OBJECT_TO_BE_BROUGHT -> NAMED_OBJECT | DET NAMED_OBJECT

V_BRING -> bring | deliver | take | carry | transport | give | hand | hand over | place | put

VP[{"action": "place", "source-location": X, "target-location": Y, "object": Z}] -> V_BRING OBJECT_TO_BE_BROUGHT[Z] from the ROOM_OR_LOCATION[X] to BRING_TARGET[Y] | V_BRING OBJECT_TO_BE_BROUGHT[Z] to BRING_TARGET[Y] from the ROOM_OR_LOCATION[X]

VP[{"action": "place", "target-location": X, "object": {"type": "reference"}}] -> V_BRING PPN_OBJECT to BRING_TARGET[X]

VP[{"action": "hand-over", "source-location": X, "target-location": Y, "object": Z}] -> V_BRING OBJECT_TO_BE_BROUGHT[Z] from the ROOM_OR_LOCATION[X] to BRING_NAME[Y] | V_BRING OBJECT_TO_BE_BROUGHT[Z] to BRING_NAME[Y] from the ROOM_OR_LOCATION[X]
VP[{"action": "hand-over", "target-location": Y, "object": Z}] -> V_BRING BRING_NAME[Y] OBJECT_TO_BE_BROUGHT[Z]
VP[{"action": "hand-over", "source-location": X, "target-location": Y, "object": Z}] -> V_BRING BRING_NAME[Y] OBJECT_TO_BE_BROUGHT[Z] from the ROOM_OR_LOCATION[X]
VP[{"action": "hand-over", "target-location": X, "object": {"type": "reference"}}] -> V_BRING PPN_OBJECT to BRING_PERSON[X]
VP[{"action": "hand-over", "target-location": X, "object": Z}] -> V_BRING OBJECT_TO_BE_BROUGHT[Z] to BRING_PERSON[X]
"""
##############################################################################
#
# SAY
#
##############################################################################

grammar += """
V_SAY -> tell | say | speak

VP[{"action": "say", "sentence": X}] -> V_SAY SAY_SENTENCE[X]
"""

grammar += '\nSAY_SENTENCE["ROBOT_NAME"] -> your name'
grammar += '\nSAY_SENTENCE["TIME"] -> the time | what time it is | what time is it'
grammar += '\nSAY_SENTENCE["my team is tech united"] -> the name of your team'
grammar += '\nSAY_SENTENCE["COUNTRY"] -> your teams country'
grammar += '\nSAY_SENTENCE["DAY_OF_MONTH"] -> the day of the month'
grammar += '\nSAY_SENTENCE["DAY_OF_WEEK"] -> the day of the week'
grammar += '\nSAY_SENTENCE["TODAY"] -> what day is today | me what day it is | the date'
grammar += '\nSAY_SENTENCE["TOMORROW"] -> what day is tomorrow'
grammar += '\nSAY_SENTENCE["JOKE"] -> a joke'
grammar += '\nSAY_SENTENCE["SOMETHING_ABOUT_SELF"] -> something about yourself'


follow_action = "follow", {"location-from": {""}, "location-to": {}, "target": {}}

##############################################################################
#
# ANSWER QUESTION
#
##############################################################################

grammar += """
VP[{"action": "answer-question"}] -> answer a question
"""

if __name__ == "__main__":
    print "GPSR Grammar:\n\n{}\n\n".format(grammar)

    from grammar_parser.cfgparser import CFGParser

    import sys
    if sys.argv[1] == "object":
        grammar_parser = CFGParser.fromstring(obj_grammar)
    elif sys.argv[1] == "location":
        grammar_parser = CFGParser.fromstring(loc_grammar)
    elif sys.argv[1] == "full":
        grammar_parser = CFGParser.fromstring(grammar)

    if len(sys.argv) > 2:
        sentence = " ".join(sys.argv[2:])
    else:
        sentence = grammar_parser.get_random_sentence("T")

    print "Parsing sentence:\n\n{}\n\n".format(sentence)

    result = grammar_parser.parse("T", sentence)

    print "Result:\n\n{}".format(result)

