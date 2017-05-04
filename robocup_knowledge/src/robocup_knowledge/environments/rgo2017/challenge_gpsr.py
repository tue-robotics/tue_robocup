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
T[{actions : <A1>}] -> C[A1]
#T[{actions : <A1, A2>}] -> C[A1] and C[A2]
#T[{actions : <A1, A2, A3>}] -> C[A1] C[A2] and C[A3]

C[{A}] -> VP[A]

##############################################################################
#
# Verbs
#
##############################################################################
VBTAKE -> bring | take
VBPLACE -> put | place
VBBRING -> bring | give
VBDELIVER -> VBBRING | deliver
VBTAKE -> get | grasp | take | pick up
VBSPEAK -> tell | say
VBGOPL["navigate-to"] -> go to | navigate to
VBGOR -> VBGOPL| enter
VBFIND -> find | locate | look for
VBGUIDE -> guide | escort | take | lead | accompany
VBFOLLOW -> follow | go after | come after

##############################################################################
#
# Bring
#
##############################################################################
VP = VBTAKE BRING_ENTITIES to the BRING_LOCATIONS
VP = VBPLACE the BRING_ENTITIES on the {placement 2}
VP = $vbbring me the $object
VP = $vbdeliver the $object to $someone
VP = $takefrom to the {placement 2}
VP = $goplace, $vbfind the $object, and ($delivme | $delivat)
VP = $goplace, $vbfind the $object, and $place

# TO BE FILLED IN BY THE KNOWLEDGE / SEMANTICS
BRING_ENTITIES -> coke | fanta

##############################################################################
#
# Navigate
#
##############################################################################
VP["action": A, "entity": X] -> VBGOPL[A] the NAV_LOCATIONS[X]

"""

for loc in common.get_locations():
    grammar += '\nNAV_LOCATIONS[{"id": %s}] -> %s' % (loc, loc)

print grammar
