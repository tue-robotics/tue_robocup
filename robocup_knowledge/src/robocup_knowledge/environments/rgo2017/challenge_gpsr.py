from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

not_understood_sentences = [
        "I'm so sorry! Can you please speak louder and slower? And wait for the ping!",
        "I am deeply sorry. Please try again, but wait for the ping!",
        "You and I have communication issues. Speak up!",
        "All this noise is messing with my audio. Try again"
    ]

initial_pose = ["initial_pose",
                "initial_pose_2"] # initial pose
starting_pose = ["gpsr_meeting_point_1",
                 "gpsr_meeting_point_2"] # Designated pose to wait for commands
exit_waypoint = ["gpsr_exit_door_1",
                 "gpsr_exit_door_2"] # Door through which to exit the arena

rooms = common.rooms + ["entrance", "exit"]

# translations = { "bookcase" : "bocase" }

grammar_target = "T"

grammar = """
##############################################################################
#
# Actions
#
##############################################################################
T[{actions : <A1>}] -> C[A1]
#T[{actions : <A1, A2>}] -> C[A1] and C[A2]
#T[{actions : <A1, A2, A3>}] -> C[A1] C[A2] and C[A3]

C[{A}] -> VP[A]

##############################################################################
#
# Verbs & shared stuff
#
##############################################################################
V_SAY -> tell | say | speak
V_TAKE -> bring | take
V_PLACE -> put | place
V_BRING -> give | bring | hand | deliver | take | carry | transport
V_TAKE -> get | grasp | take | pick up
V_SPEAK -> tell | say
V_FIND -> find | locate | look for
V_GUIDE -> guide | escort | take | lead | accompany
V_FOLLOW -> follow | go after | come after

DET -> the | a
"""

grammar += """
##############################################################################
#
# Bring
#
##############################################################################
# VP = V_TAKE BRING_ENTITIES to the BRING_LOCATIONS
# VP = V_PLACE the BRING_ENTITIES on the {placement 2}
# VP = $V_bring me the $object
# VP = $V_deliver the $object to $someone
# VP = $takefrom to the {placement 2}
# VP = $goplace, $V_find the $object, and ($delivme | $delivat)
# VP = $goplace, $V_find the $object, and $place

# TO BE FILLED IN BY THE KNOWLEDGE / SEMANTICS
BRING_ENTITIES -> coke | fanta
"""

grammar += """
##############################################################################
#
# Navigate
#
##############################################################################
V_GOPL -> go to | navigate to
V_GOR -> V_GOPL | enter

VP["action": "navigate-to", "entity": {"id": X}] -> V_GOR the ROOMS[X]
VP["action": "navigate-to", "entity": {"id": X}] -> V_GOPL the LOCATIONS[X]
"""

for room in common.rooms:
    grammar += '\nROOMS[%s] -> %s' % (room, room)

for loc in common.get_locations():
    grammar += '\nLOCATIONS[%s] -> %s' % (loc, loc)

grammar += """
##############################################################################
#
# SAY
#
##############################################################################
VP["action": "say", "sentence": X] -> SAY_SENTENCE[X]

SAY_SENTENCE["ROBOT_NAME"] -> V_SAY your name
SAY_SENTENCE["TIME"] -> V_SAY the time | V_SAY what time it is | V_SAY what time is it
SAY_SENTENCE["my team is tech united"] -> V_SAY the name of your team
SAY_SENTENCE["DAY_OF_MONTH"] -> V_SAY the day of the month
SAY_SENTENCE["DAY_OF_WEEK"] -> V_SAY the day of the week
SAY_SENTENCE["TODAY"] -> V_SAY what day is today | V_SAY me what day it is | V_SAY the date
SAY_SENTENCE["TOMORROW"] -> V_SAY what day is tomorrow
"""

if __name__ == "__main__":
    from grammar_parser.cfgparser import CFGParser
    grammar_parser = CFGParser.fromstring(grammar)
    sentence = grammar_parser.get_random_sentence("T")

    print "Parsing sentence:\n\n{}\n\n".format(sentence)

    result = grammar_parser.parse("T", sentence)

    print "Result:\n\n{}".format(result)
