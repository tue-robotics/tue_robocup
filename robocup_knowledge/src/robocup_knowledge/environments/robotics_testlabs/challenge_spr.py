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
for place in common.location_names:
    grammar += '\n PLACEMENTS_AND_BEACONS[%s] -> %s' % (place, place)

##############################################################################
#
# Predefined questions
#
##############################################################################

grammar += '''
Q["action" : "answer", "solution": "Magdeburg"] -> what city are we in
Q["action" : "answer", "solution": "Tech United"] -> what is the name of your team
Q["action" : "answer", "solution": "Chewbacca"] -> name the big hairy creature in star wars
Q["action" : "answer", "solution": "Isaac Asimov"] -> who wrote the three laws of robotics
Q["action" : "answer", "solution": "The Jetsons"] -> from what series do you know rosie the robot
Q["action" : "answer", "solution": "The Flintstones"] -> from what series do you know the baby bam bam
Q["action" : "answer", "solution": "Neo"] -> who is the main character of the matrix
Q["action" : "answer", "solution": "Peper and HSR"] -> name the two robocupathome standard platforms
Q["action" : "answer", "solution": "In my SSD"] -> where do you store your memories
Q["action" : "answer", "solution": "In Eindhoven The Netherlands"] -> where is your team located

WHATWHICH -> what | which

Q["action" : "answer", "solution": "basket"] -> WHATWHICH is the biggest object
Q["action" : "answer", "solution": "egg"] -> WHATWHICH is the smallest object
Q["action" : "answer", "solution": "pringles"] -> WHATWHICH is the biggest food
Q["action" : "answer", "solution": "egg"] -> WHATWHICH is the smallest food
Q["action" : "answer", "solution": "basket"] -> WHATWHICH is the biggest container
Q["action" : "answer", "solution": "coffecup"] -> WHATWHICH is the smallest container
Q["action" : "answer", "solution": "water"] -> WHATWHICH is the biggest drink
Q["action" : "answer", "solution": "orange drink"] -> WHATWHICH is the smallest drink
Q["action" : "answer", "solution": "paper"] -> WHATWHICH is the biggest cleaning stuff
Q["action" : "answer", "solution": "sponge"] -> WHATWHICH is the smallest cleaning stuff
Q["action" : "answer", "solution": "knife"] -> WHATWHICH is the biggest cutlery
Q["action" : "answer", "solution": "fork"] -> WHATWHICH is the smallest cutlery

Q["action" : "answer", "solution":  "the bedroom has two doors"] -> how many doors has the bedroom
Q["action" : "answer", "solution": "the dining room has two doors"] -> how many doors has the dining room
Q["action" : "answer", "solution": "in the living room there are no doors"] -> how many doors has the living room
Q["action" : "answer", "solution": "in the kitchen there are no doors"] -> how many doors has the kitchen
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

NQ["action" : "c_count", "entity" : X] -> how many people in the crowd are POSITION[X]
NQ["action" : "c_count", "entity" : W] -> how many people in the crowd are GESTURE[W]
NQ["action" : "random_gender", "entity" : X] -> the POSITION[X] person was GENDER | tell me if the POSITION[X] person was a GENDER
NQ["action" : "random_gender", "entity" : X] -> the POSITION[X] person was GENDER or GENDER | tell me if the POSITION[X] person was a GENDER or GENDER
NQ["action" : "random_gender", "entity" : W] -> tell me if the GESTURE[W] person was a GENDER
NQ["action" : "random_gender", "entity" : W] -> tell me if the GESTURE[W] person was a GENDER or GENDER
NQ["action" : "c_count", "entity" : L] -> tell me how many people were wearing COLOR[L]
'''

##############################################################################
#
# Arena questions
#
##############################################################################

grammar += '''
SEARCH -> where is | in WHATWHICH room is

Q["action" : "find_placement", "entity" : Y] -> SEARCH the PLACEMENTS_AND_BEACONS[Y]
Q["action" : "count_placement", "entity" : Y, "location" : R] -> how many PLACEMENTS_AND_BEACONS[Y] are in the ROOMS[R]
'''

##############################################################################
#
# Object / Category questions
#
##############################################################################

grammar += '''
ADJR -> smaller | bigger

Q["action" : "find_object", "entity" : O] -> where can i find DET OBJECT_NAMES[O]
Q["action" : "find_category", "entity" : C] -> where can i find DET OBJECT_CATEGORIES[C]
Q["action" : "return_category", "entity" : O] -> to WHATWHICH category belong the OBJECT_NAMES[O]
Q["action" : "return_color", "entity" : O] -> whats the color of the OBJECT_NAMES[O]
Q["action" : "compare_category", "entity_a" : O, "entity_b" : A] -> do the OBJECT_NAMES[O] and OBJECT_NAMES[A] belong to the same category
Q["action" : "count_object", "entity" : C] -> how many OBJECT_CATEGORIES[C] there are

NQ["action" : "count_object", "entity" : C, "location" : Y] -> how many OBJECT_CATEGORIES[C] are in the PLACEMENTS_AND_BEACONS[Y]
NQ["action" : "count_object", "entity" : O, "location" : Y] -> how many OBJECT_NAMES[O] are in the PLACEMENTS_AND_BEACONS[Y]
NQ["action" : "category_at_loc", "location" : Y] -> what objects are stored in the PLACEMENTS_AND_BEACONS[Y]

NQ["action" : "compare", "entity_a" : O, "entity_b" : A] -> between the OBJECT_NAMES[O] and OBJECT_NAMES[A] which one is ADJR
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

GENDER['male'] -> male
GENDER['female'] -> female
GENDER['man'] -> man
GENDER['woman'] -> woman
GENDER['boy'] -> boy
GENDER['girl'] -> girl

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
