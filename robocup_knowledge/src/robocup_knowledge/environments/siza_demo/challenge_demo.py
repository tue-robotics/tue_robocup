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
T[A] -> C[A] | COURTESY C[A]

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
    grammar += '\nNAMED_OBJECT[%s] -> %s' % (obj, obj)

for loc in common.get_locations(pick_location=True, place_location=True):
    grammar += '\nMANIPULATION_AREA_LOCATION[%s] -> MANIPULATION_AREA_DESCRIPTION the %s' % (loc, loc)

for cat in common.object_categories:
    grammar += '\nOBJECT_CATEGORY[%s] -> %s' % (cat, cat)

for name in common.names:
    grammar += '\nNAMED_PERSON[%s] -> %s' % (name, name)

###############################################################################
#
# Demo presentation
#
###############################################################################

grammar += """
V_PRESENT -> introduce yourself | present yourself | perform a demonstration | give a presentation
ENGLISH['en'] -> english
DUTCH['nl'] -> dutch
LANGUAGE[X] -> ENGLISH[X] | DUTCH[X]
VP["action": "demo-presentation", 'language': 'en'] -> V_PRESENT
VP["action": "demo-presentation", "language": X] -> V_PRESENT in LANGUAGE[X]
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
# Find objects
#
###############################################################################

grammar += """
V_FIND -> find | locate | look for | pinpoint | spot
V_FIND_PERSON -> meet | V_FIND

OBJECT_TO_BE_FOUND -> NAMED_OBJECT | OBJECT_CATEGORY
PERSON_TO_BE_FOUND -> DET person | DET woman | DET man | someone | me

VP["action": "find", "object": {"type": X}, "location": {"id": Y}] -> V_FIND DET OBJECT_TO_BE_FOUND[X] MANIPULATION_AREA_LOCATION[Y]
VP[{"action": "find", "object": X, "source-location": Y}] -> V_FIND DET OBJECT_TO_BE_FOUND[X] in the ROOM[Y]
VP["action": "find", "object": {"type": X}] -> V_FIND DET OBJECT_TO_BE_FOUND[X]

VP[{"action": "find", "object": X, "source-location": Y}] -> V_FIND PERSON_TO_BE_FOUND[X] in the ROOM[Y]
VP[{"action": "find", "object": X, "source-location": Y}] -> V_FIND PERSON_TO_BE_FOUND[X] near the LOCATION[Y]
VP[{"action": "find", "object": X}] -> V_FIND DET PERSON_TO_BE_FOUND[X]
"""

###############################################################################
#
# Navigate
#
###############################################################################

grammar += """
V_GOPL -> go to | navigate to | drive to

VP["action": "navigate-to", "object": {"id": X}] -> V_GOPL the LOCATION[X]
"""

###############################################################################
#
# Inspect
#
###############################################################################

grammar += """

VP["action": "inspect", "entity": {"id": X}] -> inspect the LOCATION[X]
"""

###############################################################################
#
# BRING
#
###############################################################################

grammar += """
OBJECT_TO_BE_BROUGHT -> NAMED_OBJECT | DET NAMED_OBJECT

V_BRING -> bring | deliver | take | carry | transport | give | hand | hand over

VP["action": "place", "source-location": {"id": X}, "target-location": {"id": Y}, "object": {"type": Z}] -> V_BRING OBJECT_TO_BE_BROUGHT[Z] from the LOCATION[X] to the LOCATION[Y]
VP["action": "hand-over", "source-location": {"id": X}, "target-location": {"id": "operator"}, "object": {"type": Z}] -> V_BRING me OBJECT_TO_BE_BROUGHT[Z] from the LOCATION[X] | V_BRING OBJECT_TO_BE_BROUGHT[Z] from the LOCATION[X] to me
"""

##############################################################################
#
# SAY
#
##############################################################################

grammar += """
V_SAY -> tell | say

VP["action": "say", "sentence": X] -> V_SAY SAY_SENTENCE[X]
"""

grammar += '\nSAY_SENTENCE["ROBOT_NAME"] -> your name'
grammar += '\nSAY_SENTENCE["TIME"] -> the time | what time it is | what time is it'
grammar += '\nSAY_SENTENCE["my team is tech united"] -> the name of your team'
grammar += '\nSAY_SENTENCE["DAY_OF_MONTH"] -> the day of the month'
grammar += '\nSAY_SENTENCE["DAY_OF_WEEK"] -> the day of the week'
grammar += '\nSAY_SENTENCE["TODAY"] -> what day is today | me what day it is | the date'
grammar += '\nSAY_SENTENCE["JOKE"] -> a joke'

###############################################################################
#
# Test
#
###############################################################################

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
