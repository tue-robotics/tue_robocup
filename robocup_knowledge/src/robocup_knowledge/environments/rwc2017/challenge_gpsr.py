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
T[{actions : <A1>}] -> C[A1]
T[{actions : <A1, A2>}] -> C[A1] and C[A2]
T[{actions : <A1, A2, A3>}] -> C[A1] C[A2] and C[A3]

C[{A}] -> VP[A]
"""

##############################################################################
#
# Verbs & shared stuff
#
##############################################################################

grammar += """
PPN_OBJECT[reference] -> it | them
PPN_PERSON -> him | her | them

DET -> the | a | an | some
NUMBER -> one | two | three
MANIPULATION_AREA_DESCRIPTION -> on top of | at | in | on | from
"""

for room in common.location_rooms:
    grammar += '\nROOM[%s] -> %s' % (room, room)

for loc in common.get_locations():
    grammar += '\nLOCATION[%s] -> %s' % (loc, loc)

grammar += '\n ROOM_OR_LOCATION[X] -> ROOM[X] | LOCATION[X]'

for obj in common.object_names:
    grammar += '\nNAMED_OBJECT[%s] -> %s' % (obj, obj)

for loc in common.get_locations(pick_location=True, place_location=True):
    grammar += '\nMANIPULATION_AREA_LOCATION[%s] -> MANIPULATION_AREA_DESCRIPTION the %s' % (loc, loc)

for cat in common.object_categories:
    grammar += '\nOBJECT_CATEGORY[%s] -> %s' % (cat, cat)

for name in common.names:
    grammar += '\nNAMED_PERSON[%s] -> %s' % (name, name)

###############################################################################
#
# FIND
#
###############################################################################

grammar += """
V_FIND -> find | locate | look for | meet

OBJECT_TO_BE_FOUND -> NAMED_OBJECT | OBJECT_CATEGORY
PERSON_TO_BE_FOUND -> DET person | DET woman | DET man | NAMED_PERSON | someone

VP["action": "find", "object": {"type": X}, "location": {"id": Y}] -> V_FIND DET OBJECT_TO_BE_FOUND[X] in the ROOM[Y]
VP["action": "find", "object": {"type": X}, "location": {"id": Y}] -> V_FIND DET OBJECT_TO_BE_FOUND[X] MANIPULATION_AREA_LOCATION[Y]

VP["action": "find", "object": {"type": "person", "id": X}, "location": {"id": Y}] -> V_FIND PERSON_TO_BE_FOUND[X] in the ROOM[Y]
VP["action": "find", "object": {"type": "person", "id": X}, "location": {"id": Y}] -> V_FIND PERSON_TO_BE_FOUND[X] near the LOCATION[Y]

VP["action": "find", "object": {"type": X}] -> V_FIND DET OBJECT_TO_BE_FOUND[X]
VP["action": "find", "object": {"type": person, "id": X}] -> V_FIND DET PERSON_TO_BE_FOUND[X]
"""

###############################################################################
#
# Navigate
#
###############################################################################

grammar += """
V_GOPL -> go to | navigate to
V_GOR -> V_GOPL | enter

VP["action": "navigate-to", "object": {"id": X}] -> V_GOR the ROOM[X]
VP["action": "navigate-to", "object": {"id": X}] -> V_GOPL the LOCATION[X]
"""

###############################################################################
#
# Pick-up
#
###############################################################################

grammar += """
V_PICKUP -> get | grasp | take | pick up | grab

VP["action": "pick-up", "object": {"type": X}, "location": {"id": Y}] -> V_PICKUP DET NAMED_OBJECT[X] MANIPULATION_AREA_LOCATION[Y]
"""

###############################################################################
#
# Place
#
###############################################################################

grammar += """
V_PLACE -> put | place

VP["action": "place", "object": {"type": "reference"}, "location": {"id": Y}] -> V_PLACE PPN_OBJECT MANIPULATION_AREA_LOCATION[Y]
"""

###############################################################################
#
# BRING
#
###############################################################################

for room in common.location_rooms:
    grammar += '\nBRING_ROOM[{"type": room, "id": %s}] -> %s' % (room, room)

for loc in common.get_locations():
    grammar += '\nBRING_LOCATION[{"type": furniture, "id": %s}] -> %s' % (loc, loc)

grammar += '\n BRING_ROOM_OR_LOCATION[X] -> BRING_ROOM[X] | BRING_LOCATION[X]'

grammar += """
INAT -> in | at

BRING_PERSON_AT_LOCATION[{"type": person, "id": X, "loc": Y}] -> NAMED_PERSON[X] INAT the ROOM_OR_LOCATION[Y]
BRING_OPERATOR[{"type": person, "id": operator}] -> me
BRING_PERSON -> BRING_OPERATOR | BRING_PERSON_AT_LOCATION

BRING_TARGET[X] -> BRING_PERSON[X] | the BRING_LOCATION[X]

OBJECT_TO_BE_BROUGHT -> NAMED_OBJECT | DET NAMED_OBJECT | PPN_OBJECT

V_BRING -> bring | deliver | take | give | get
"""

# Bring <object> from the <location> to the <location>
# Bring <object> to the <location> from the <location>
grammar += """
VP["action": "bring", "source-location": X, "target-location": Y, "object": {"type": Z}] -> V_BRING OBJECT_TO_BE_BROUGHT[Z] from the ROOM_OR_LOCATION[X] to BRING_TARGET[Y] | V_BRING OBJECT_TO_BE_BROUGHT[Z] to BRING_TARGET[Y] from the ROOM_OR_LOCATION[X]
"""

# Bring to <person> the <object> | Bring <object> to <person> | bring <person> the <object>
grammar += """
VP["action": "bring", "target-location": Y, "object": {"type": Z}] -> V_BRING to BRING_TARGET[Y] OBJECT_TO_BE_BROUGHT[Z] | V_BRING OBJECT_TO_BE_BROUGHT[Z] to BRING_TARGET[Y] | V_BRING BRING_PERSON[Y] OBJECT_TO_BE_BROUGHT[Z]
"""

# Bring to <person> the <object> from the <location>
grammar += """
VP["action": "bring", "source-location": X, "target-location": Y, "object": {"type": Z}] -> V_BRING to BRING_TARGET[Y] OBJECT_TO_BE_BROUGHT[Z] from the ROOM_OR_LOCATION[X] | V_BRING BRING_TARGET[Y] OBJECT_TO_BE_BROUGHT[Z] from the ROOM_OR_LOCATION[X]
"""

# Place the <object> on the <location>
grammar += """
VP["action": "bring", "object": {"type": Z}, "target-location": Y] -> V_PLACE OBJECT_TO_BE_BROUGHT[Z] MANIPULATION_AREA_DESCRIPTION the BRING_LOCATION[Y]
"""


##############################################################################
#
# SAY
#
##############################################################################

grammar += """
V_SAY -> tell | say | speak

VP["action": "say", "sentence": X] -> V_SAY SAY_SENTENCE[X]
VP["action": "say", "sentence": X, "target-person": Y] -> V_SAY SAY_SENTENCE[X] to BRING_PERSON_AT_LOCATION[Y]
"""

grammar += '\nSAY_SENTENCE["ROBOT_NAME"] -> your name'
grammar += '\nSAY_SENTENCE["TIME"] -> the time | what time it is | what time is it'
grammar += '\nSAY_SENTENCE["my team is tech united"] -> the name of your team'
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
VP["action": "answer-question"] -> answer a question
VP["action": "answer-question", "target-person": X] -> answer a question to BRING_PERSON_AT_LOCATION[X]
"""

##############################################################################
#
# FIND OUT AND REPORT
#
##############################################################################

grammar += """
PROPERTY["number"] -> number
PROPERTY["name"] -> name

INVESTIGATION_LOCATION[X] -> MANIPULATION_AREA_LOCATION[X] | in the ROOM

VP["action": "find-out-and-report", "property": X, "object": {"type": Y}] -> tell me the PROPERTY[X] of OBJECT_TO_BE_FOUND[Y] INVESTIGATION_LOCATION[Z]
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


##############################################################################
#
# Question grammar
#
##############################################################################

question_grammar_target = "T"

question_grammar = """
T[{actions : <A1>}] -> C[A1]

C[{A}] -> Q[A]
"""

# Predefined questions
question_grammar += '''

WHATWHICH -> what | which

BIGGEST_ADJ -> biggest | heaviest
SMALLEST_ADJ -> smallest | lightest

Q["action" : "answer", "solution": "bread"] -> WHATWHICH is the BIGGEST_ADJ object
Q["action" : "answer", "solution": "chopsticks"] -> WHATWHICH is the SMALLEST_ADJ object
Q["action" : "answer", "solution": "bread"] -> WHATWHICH is the BIGGEST_ADJ food
Q["action" : "answer", "solution": "onion"] -> WHATWHICH is the SMALLEST_ADJ food
Q["action" : "answer", "solution": "plate"] -> WHATWHICH is the BIGGEST_ADJ container
Q["action" : "answer", "solution": "soup_container"] -> WHATWHICH is the SMALLEST_ADJ container
Q["action" : "answer", "solution": "green_tea"] -> WHATWHICH is the BIGGEST_ADJ drink
Q["action" : "answer", "solution": "coke"] -> WHATWHICH is the SMALLEST_ADJ drink
Q["action" : "answer", "solution": "hair_spray"] -> WHATWHICH is the BIGGEST_ADJ cleaning stuff
Q["action" : "answer", "solution": "moisturizer"] -> WHATWHICH is the SMALLEST_ADJ cleaning stuff
Q["action" : "answer", "solution": "spoon"] -> WHATWHICH is the BIGGEST_ADJ cutlery
Q["action" : "answer", "solution": "chopsticks"] -> WHATWHICH is the SMALLEST_ADJ cutlery

Q["action" : "answer", "solution":  "the bedroom has one door"] -> how many doors has the bedroom
Q["action" : "answer", "solution": "the entrance has one door"] -> how many doors has the entrance
Q["action" : "answer", "solution": "the living room has one door"] -> how many doors has the living_room
Q["action" : "answer", "solution": "the kitchen has one door"] -> how many doors has the kitchen
Q["action" : "answer", "solution": "the corridor has zero doors"] -> how many doors has the corridor
Q["action" : "answer", "solution": "the balcony has zero doors"] -> how many doors has the balcony
'''

