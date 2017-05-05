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

# rooms = common.rooms + ["entrance", "exit"]

# translations = { "bookcase" : "bocase" }

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
V_GUIDE -> guide | escort | take | lead | accompany

DET -> the | a
MANIPULATION_AREA_DESCRIPTIONS -> on top of | at | in | on
"""

for room in common.location_rooms:
    grammar += '\nROOMS[%s] -> %s' % (room, room)
for loc in common.get_locations():
    grammar += '\nLOCATIONS[%s] -> %s' % (loc, loc)
grammar += '\n ROOMS_AND_LOCATIONS[X] -> ROOMS[X] | LOCATIONS[X]'
for obj in common.object_names:
    grammar += '\nOBJECT_NAMES[%s] -> %s' % (obj, obj)
for loc in common.get_locations(pick_location=True, place_location=True):
    grammar += '\nMANIPULATION_AREA_LOCATIONS[%s] -> MANIPULATION_AREA_DESCRIPTIONS the %s' % (loc, loc)
for cat in common.object_categories:
    grammar += '\nOBJECT_NAMES[%s] -> %s' % (cat, cat)

###############################################################################
#
# FIND
#
###############################################################################

grammar += """
V_FIND -> find | locate | look for

VP["action": "find", "object": {"type": X}, "location": {"id": Y}] -> V_FIND DET OBJECT_NAMES[X] in the ROOMS[Y]
VP["action": "find", "object": {"type": "person"}, "location": {"id": Y}] -> V_FIND FIND_PERSONS in the ROOMS[Y]
VP["action": "find", "object": {"type": "person"}, "location": {"id": Y}] -> V_FIND FIND_PERSONS near the LOCATIONS[Y]
VP["action": "find", "object": {"type": X}, "location": {"id": Y}] -> V_FIND DET OBJECT_NAMES[X] MANIPULATION_AREA_LOCATIONS[Y]
"""

grammar += '\nFIND_PERSONS -> DET person'
grammar += '\nFIND_PERSONS -> DET women'
grammar += '\nFIND_PERSONS -> DET man'
for name in common.names:
    grammar += '\nFIND_PERSONS -> %s' % name

###############################################################################
#
# Navigate
#
###############################################################################

grammar += """
V_GOPL -> go to | navigate to
V_GOR -> V_GOPL | enter

VP["action": "navigate-to", "object": {"id": X}] -> V_GOR the ROOMS[X]
VP["action": "navigate-to", "object": {"id": X}] -> V_GOPL the LOCATIONS[X]
"""

###############################################################################
#
# Pick-up
#
###############################################################################

grammar += """
V_PICKUP -> get | grasp | take | pick up

VP["action": "find", "object": {"type": X}, "location": {"id": Y}] -> V_PICKUP DET OBJECT_NAMES[X] MANIPULATION_AREA_LOCATIONS[Y]
"""

###############################################################################
#
# Place
#
###############################################################################

grammar += """
V_PLACE -> put | place

VP["action": "place", "object": {"type": X}, "location": {"id": Y}] -> V_PLACE DET OBJECT_NAMES[X] MANIPULATION_AREA_LOCATIONS[Y]
"""

###############################################################################
#
# Follow
#
###############################################################################

grammar += """
V_FOLLOW -> follow | go after | come after

VP["action": "follow", "location-from": {"id": X}, "location-to": {"id": Y}, "target": {"id": "operator"}] -> V_FOLLOW me from the ROOMS_AND_LOCATIONS[X] to the ROOMS_AND_LOCATIONS[Y]
VP["action": "follow", "location-to": {"id": X}, "target": {"id": "operator"}] -> V_FOLLOW me to the ROOMS_AND_LOCATIONS[X]
VP["action": "follow", "target": {"id": "operator"}] -> V_FOLLOW me

VP["action": "follow", "location-from": {"id": X}, "location-to": {"id": Y}, "target": {"id": Z}] -> V_FOLLOW FOLLOW_PERSONS[Z] from the ROOMS_AND_LOCATIONS[X] to the ROOMS_AND_LOCATIONS[Y]
VP["action": "follow", "location-to": {"id": X}, "target": {"id": Z}] -> V_FOLLOW FOLLOW_PERSONS[Z] to the ROOMS_AND_LOCATIONS[X]
VP["action": "follow", "target": {"id": Z}] -> V_FOLLOW FOLLOW_PERSONS[Z]
"""

grammar += '\nFOLLOW_PERSONS[the person] -> DET person'
grammar += '\nFOLLOW_PERSONS[the woman] -> DET woman'
grammar += '\nFOLLOW_PERSONS[the man] -> DET man'
for name in common.names:
    grammar += '\nFOLLOW_PERSONS[%s] -> %s' % (name, name)

###############################################################################
#
# BRING
#
###############################################################################

grammar += """
V_BRING -> bring | deliver | take | carry | transport
V_BRING_PERSON -> V_BRING | give | hand | hand over

VP["action": "bring", "source-location": {"id": X}, "target-location": {"id": Y}, "object": {"type": Z}] -> V_BRING DET OBJECT_NAMES[Z] from the ROOMS_AND_LOCATIONS[X] to the ROOMS_AND_LOCATIONS[Y]
VP["action": "bring", "source-location": {"id": X}, "target-location": {"type": "person"}, "object": {"type": Z}] -> V_BRING DET OBJECT_NAMES[Z] from the ROOMS_AND_LOCATIONS[X] to BRING_PERSONS
VP["action": "bring", "source-location": {"id": X}, "target-location": {"type": "person", "id": "operator"}, "object": {"type": Z}] -> V_BRING DET OBJECT_NAMES[Z] from the ROOMS_AND_LOCATIONS[X] to me
"""

for name in common.names:
    grammar += '\nBRING_PERSONS -> %s' % name

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

if __name__ == "__main__":
    print "GPSR Grammar:\n\n{}\n\n".format(grammar)

    from grammar_parser.cfgparser import CFGParser
    grammar_parser = CFGParser.fromstring(grammar)
    sentence = grammar_parser.get_random_sentence("T")

    print "Parsing sentence:\n\n{}\n\n".format(sentence)

    result = grammar_parser.parse("T", sentence)

    print "Result:\n\n{}".format(result)


follow_acton = "follow", {"location-from": {""}, "location-to": {}, "target": {}}
