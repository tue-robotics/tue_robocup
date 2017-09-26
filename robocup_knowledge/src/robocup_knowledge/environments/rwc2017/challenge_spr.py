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

C[{A}] -> Q[A]
"""

# Q questions are implemented already in the challenge.
# NQ questions are still not implemented, so the robot should not understand them. Future tasks! :)

##############################################################################
#
# Verbs & shared stuff
#
##############################################################################

grammar += """
V_GUIDE -> guide | escort | take | lead | accompany

DET -> the | a | an | some
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
    grammar += '\nOBJECT_CATEGORIES[%s] -> %s' % (cat, cat)
grammar += '\nOBJECT_CATEGORIES[objects] -> objects'
for place in common.location_names:
    grammar += '\n PLACEMENTS_AND_BEACONS[%s] -> %s' % (place, place)

##############################################################################
#
# Predefined questions
#
##############################################################################

grammar += '''
Q["action" : "answer", "solution": "Nagoya"] -> what city are we in
Q["action" : "answer", "solution": "Tech united"] -> what is the name of your team
Q["action" : "answer", "solution": "31"] -> how many teams participate in robocup at home this year
Q["action" : "answer", "solution": "Hillary Clinton"] -> who won the popular vote in the us election
Q["action" : "answer", "solution": "Mount Fuji"] -> what is the highest mountain in japan
Q["action" : "answer", "solution": "Pepper and HSR"] -> name the two robocup at home standard platforms
Q["action" : "answer", "solution": "Domestic Standard Platform League"] -> what does dspl stand for
Q["action" : "answer", "solution": "Social Standard Platform League"] -> what does sspl stand for
Q["action" : "answer", "solution": "SoftBank"] -> who did alphabet sell boston dynamics to
Q["action" : "answer", "solution": "over 410000 square metres"] -> nagoya has one of the largest train stations in the world. how large is it
Q["action" : "answer", "solution": "My team is located in Eindhoven"] -> where is your team located
Q["action" : "answer", "solution": "George Lucas"] -> who created star wars
Q["action" : "answer", "solution": "Sponge Bob Squarepants"] -> who lives in a pineapple under the sea
Q["action" : "answer", "solution": "the inventor of the first compiler"] -> who is grace hopper
Q["action" : "answer", "solution": "the inventor of the first compiler"] -> what invented grace hopper

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

##############################################################################
#
# Counting People
#
##############################################################################

grammar += '''
Q["action" : "count", "entity" : P] -> how many PEOPLE[P] are in the crowd | tell me the number of PEOPLE[P] in the crowd
'''

##############################################################################
#
# Crowd position / gesture questions
#
##############################################################################

grammar += '''

Q["action" : "c_count", "entity" : X] -> how many people in the crowd are POSITION[X]
Q["action" : "c_count", "entity" : W] -> how many people in the crowd are GESTURE[W]
Q["action" : "random_gender", "entity" : X] -> the POSITION[X] person was GENDER | tell me if the POSITION[X] person was a GENDER
Q["action" : "random_gender", "entity" : X] -> the POSITION[X] person was GENDER or GENDER | tell me if the POSITION[X] person was a GENDER or GENDER | was the POSITION[X] person GENDER or GENDER
Q["action" : "random_gender", "entity" : W] -> tell me if the GESTURE[W] person was a GENDER
Q["action" : "random_gender", "entity" : W] -> tell me if the GESTURE[W] person was a GENDER or GENDER
Q["action" : "c_count", "entity" : L] -> tell me how many people were wearing COLOR[L]
'''

##############################################################################
#
# Arena questions
#
##############################################################################

grammar += '''
SEARCH -> where is | in WHATWHICH room is | where is located

Q["action" : "find_placement", "entity" : Y] -> SEARCH the PLACEMENTS_AND_BEACONS[Y]
Q["action" : "find_placement", "entity" : Y] -> SEARCH the PLACEMENTS_AND_BEACONS[Y]
Q["action" : "count_placement", "entity" : Y, "location" : R] -> how many PLACEMENTS_AND_BEACONS[Y] are in the ROOMS[R]
Q["action" : "count_placement", "entity" : Y, "location" : R] -> how many OBJECT_CATEGORIES[Y] are in the ROOMS[R]
Q["action" : "count_placement", "entity" : Y, "location" : R] -> how many OBJECT_CATEGORIES[Y] are in the PLACEMENTS_AND_BEACONS[R]
'''

##############################################################################
#
# Object / Category questions
#
##############################################################################

grammar += '''
ADJR -> smaller | bigger | heavier | lighter

Q["action" : "find_object", "entity" : O] -> where can i find DET OBJECT_NAMES[O]
Q["action" : "find_category", "entity" : C] -> where can i find DET OBJECT_CATEGORIES[C]
Q["action" : "return_category", "entity" : O] -> to WHATWHICH category belong the OBJECT_NAMES[O]
Q["action" : "return_color", "entity" : O] -> whats the color of the OBJECT_NAMES[O]
Q["action" : "compare_category", "entity_a" : O, "entity_b" : A] -> do the OBJECT_NAMES[O] and OBJECT_NAMES[A] belong to the same category
Q["action" : "count_object", "entity" : C] -> how many OBJECT_CATEGORIES[C] there are

Q["action" : "count_object", "entity" : C, "location" : Y] -> how many OBJECT_CATEGORIES[C] are in the PLACEMENTS_AND_BEACONS[Y]
Q["action" : "count_object", "entity" : O, "location" : Y] -> how many OBJECT_NAMES[O] are in the PLACEMENTS_AND_BEACONS[Y]
Q["action" : "category_at_loc", "location" : Y] -> what objects are stored in the PLACEMENTS_AND_BEACONS[Y]

Q["action" : "compare", "entity_a" : O, "entity_b" : A] -> between the OBJECT_NAMES[O] and OBJECT_NAMES[A] which one is ADJR
'''

##############################################################################
#
# Data
#
##############################################################################

grammar += '''

PEOPLE['people'] -> people
PEOPLE['children'] -> children
PEOPLE['adults'] -> adults
PEOPLE['elders'] -> elders
PEOPLE['males'] -> males
PEOPLE['females'] -> females
PEOPLE['men'] -> men
PEOPLE['women'] -> women
PEOPLE['boys'] -> boys
PEOPLE['girls'] -> girls

GENDER -> male | female | man | woman | boy | girl

POSITION['standing'] -> standing
POSITION['sitting'] -> sitting
POSITION['lying'] -> lying

GESTURE['waving'] -> waving
GESTURE['rise_l_arm'] -> rising left arm
GESTURE['rise_r_arm'] -> rising right arm
GESTURE['left'] -> pointing left
GESTURE['right'] -> pointing right

COLOR['red'] -> red
COLOR['blue'] -> blue
COLOR['white'] -> white
COLOR['black'] -> black
COLOR['green'] -> green
COLOR['yellow'] -> yellow
'''

if __name__ == "__main__":
    print "\n\n{}\n\n".format(grammar)
