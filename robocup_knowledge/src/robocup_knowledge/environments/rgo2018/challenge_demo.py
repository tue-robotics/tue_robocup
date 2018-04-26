from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

not_understood_sentences = [
        "I'm so sorry! Can you please speak louder and slower? And wait for the ping!",
        "I am deeply sorry. Please try again, but wait for the ping!",
        "You and I have communication issues. Speak up!",
        "All this noise is messing with my audio. Try again"
    ]

grammar_target = "T"

##############################################################################
#
# Actions
#
##############################################################################

grammar = """
T[{actions : <A1>}] -> C[A1]

C[{A}] -> VP[A]
"""

##############################################################################
#
# Verbs & shared stuff
#
##############################################################################

grammar += """
V_GUIDE -> guide | escort | take | lead | accompany

DET -> the | a | an | some
NUMBER -> one | two | three
MANIPULATION_AREA_DESCRIPTION -> on top of | at | in | on | from
"""

for loc in common.get_locations():
    grammar += '\nLOCATION[%s] -> %s' % (loc, loc)

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
# Find objects
#
###############################################################################

grammar += """
V_FIND -> find | locate | look for

VP["action": "find", "object": {"type": X}, "location": {"id": Y}] -> V_FIND DET NAMED_OBJECT[X] MANIPULATION_AREA_LOCATION[Y]
VP["action": "find", "object": {"type": X}] -> V_FIND DET NAMED_OBJECT[X]
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

VP["action": "bring", "source-location": {"id": X}, "target-location": {"id": Y}, "object": {"type": Z}] -> V_BRING OBJECT_TO_BE_BROUGHT[Z] from the LOCATION[X] to the LOCATION[Y]
VP["action": "bring", "source-location": {"id": X}, "target-location": {"type": "person", "id": "operator"}, "object": {"type": Z}] -> V_BRING me OBJECT_TO_BE_BROUGHT[Z] from the LOCATION[X] | V_BRING OBJECT_TO_BE_BROUGHT[Z] from the LOCATION[X] to me
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
# Sound Source Localization (SSL)
#
###############################################################################

grammar += """
VP["action": "turn-toward-sound", "duration": "30"] -> show your sound source localization | look at me when i am talking to you
"""

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
