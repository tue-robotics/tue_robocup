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
T[{actions : <A>}] -> C[A] | amigo C[A]

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
# Demo
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
# Navigate
#
###############################################################################

grammar += """
V_GOPL -> go to | navigate to
V_GOR -> V_GOPL | enter

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

VP["action": "place", "object": {"type": X}, "location": {"id": Y}] -> V_PLACE DET NAMED_OBJECT[X] MANIPULATION_AREA_LOCATION[Y]
"""

###############################################################################
#
# Follow
#
###############################################################################

grammar += """
V_FOLLOW -> follow | come after

VP["action": "follow", "target": {"id": "operator"}] -> V_FOLLOW me
"""

###############################################################################
#
# BRING
#
###############################################################################

grammar += """
OPERATOR[operator] -> me
BRING_NAME -> OPERATOR | BRING_PERSON

BRING_TARGET[{"id": X, "type": person}] -> BRING_NAME[X]
BRING_TARGET[{"id": X}] -> the LOCATION[X]

OBJECT_TO_BE_BROUGHT -> NAMED_OBJECT | DET NAMED_OBJECT

V_BRING -> bring | deliver | take | carry | transport | give | hand | hand over

VP["action": "bring", "source-location": {"id": X}, "target-location": Y, "object": {"type": Z}] -> V_BRING OBJECT_TO_BE_BROUGHT[Z] from the LOCATION[X] to BRING_TARGET[Y]
VP["action": "bring", "target-location": {"type": "person", "id": Y}, "object": {"type": Z}] -> V_BRING BRING_NAME[Y] OBJECT_TO_BE_BROUGHT[Z]
VP["action": "bring", "source-location": {"id": X}, "target-location": {"type": "person", "id": Y}, "object": {"type": Z}] -> V_BRING BRING_NAME[Y] OBJECT_TO_BE_BROUGHT[Z] from the LOCATION[X]
"""

for name in common.names:
    grammar += '\nBRING_PERSON[%s] -> %s' % (name, name)

##############################################################################
#
# SAY
#
##############################################################################

grammar += """
V_SAY -> tell | say | speak

VP["action": "say", "sentence": X] -> V_SAY SAY_SENTENCE[X]
"""

grammar += '\nSAY_SENTENCE["ROBOT_NAME"] -> your name'
grammar += '\nSAY_SENTENCE["TIME"] -> the time | what time it is | what time is it'
grammar += '\nSAY_SENTENCE["my team is tech united"] -> the name of your team'
grammar += '\nSAY_SENTENCE["DAY_OF_MONTH"] -> the day of the month'
grammar += '\nSAY_SENTENCE["DAY_OF_WEEK"] -> the day of the week'
grammar += '\nSAY_SENTENCE["TODAY"] -> what day is today | me what day it is | the date'
grammar += '\nSAY_SENTENCE["TOMORROW"] -> what day is tomorrow'
grammar += '\nSAY_SENTENCE["JOKE"] -> a joke'


##############################################################################
#
# ANSWER QUESTION
#
##############################################################################

grammar += """
VP["action": "answer-question"] -> answer a question
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
