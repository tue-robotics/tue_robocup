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


##############################################################################
#
# Verbs & shared stuff
#
##############################################################################

grammar += """
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


Q["action" : "answer", "solution": "the scientific study of robot"] -> what is robotics
Q["action" : "answer", "solution": "joseph engelberger"] -> who is considered as the father of industrial robot	
Q["action" : "answer", "solution": "a m l a manufacturing language"] -> give one example for a computer programming language that can be used for robot programming
Q["action" : "answer", "solution": "heavy investment"] -> what is the major disadvantage of using a robot
Q["action" : "answer", "solution": "lisp and prolog"] -> name the language used in expert system
Q["action" : "answer", "solution": "knowledge base"] -> name one of the most important parts in expert system
Q["action" : "answer", "solution": "mycin"] -> which system was designed for diagnosis and therapy recommendation for infectious disease
Q["action" : "answer", "solution": "human intelligence have certain limit in speed and accuracy it is interrupted due to the lack of presence of mind mood etc but a i systems have their own superb abilities"] -> what is the importance of a i
Q["action" : "answer", "solution": "tim burners lee"] -> weaving the web was written by
Q["action" : "answer", "solution": "trial test of a computer or software before the commercial launch"] -> what is beta test
Q["action" : "answer", "solution": "portable document format"] -> what is the extension of pdf
Q["action" : "answer", "solution": "relational data base management system"] -> expand rdbms
Q["action" : "answer", "solution": "charles babbage"] -> difference engine was developed by
Q["action" : "answer", "solution": "google"] -> orkut dot com is now owned by
Q["action" : "answer", "solution": "google"] -> orkut com is now owned by
Q["action" : "answer", "solution": "intel 4004"] -> worlds first microprocessor is
Q["action" : "answer", "solution": "structured query language"] -> what is sql
Q["action" : "answer", "solution": "short message service"] -> what is the expansion of sms	 
Q["action" : "answer", "solution": "i b m"] -> which it companys nickname is the big blue
Q["action" : "answer", "solution": "institute of electric and electronic engineers"] -> what is the full form of ieee
Q["action" : "answer", "solution": "raymond samuel tomlinson"] -> email was developed by
Q["action" : "answer", "solution": "net citizen citizen who uses internet"] -> who is netizen
Q["action" : "answer", "solution": "fake antivirus softwares"] -> what is scareware

WHATWHICH -> what | which

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
# Arena questions
#
##############################################################################

grammar += '''
SEARCH -> where is located | in WHATWHICH room is

Q["action" : "find_placement", "entity" : Y] -> SEARCH the PLACEMENTS_AND_BEACONS[Y]
Q["action" : "count_placement", "entity" : Y, "location" : R] -> how many PLACEMENTS_AND_BEACONS[Y] are in the ROOMS[R]
Q["action" : "answer", "solution": "0"] -> how many doors has the living room
Q["action" : "answer", "solution": "2"] -> how many doors has the dining room
Q["action" : "answer", "solution": "2"] -> how many doors has the bed room
Q["action" : "answer", "solution": "0"] -> how many doors has the kitchen
'''

##############################################################################
#
# Object / Category questions
#
##############################################################################

grammar += '''
ADJR -> smaller | bigger 
#| heavier | lighter

Q["action" : "find_object", "entity" : O] -> where can i find DET OBJECT_NAMES[O]
Q["action" : "find_category", "entity" : C] -> where can i find DET OBJECT_CATEGORIES[C]
Q["action" : "return_category", "entity" : O] -> to WHATWHICH category belong the OBJECT_NAMES[O]
Q["action" : "return_color", "entity" : O] -> whats the color of the OBJECT_NAMES[O]
Q["action" : "compare_category", "entity_a" : O, "entity_b" : A] -> do the OBJECT_NAMES[O] and OBJECT_NAMES[A] belong to the same category
Q["action" : "count_object", "entity" : C] -> how many OBJECT_CATEGORIES[C] there are

NQ["action" : "count_placement", "entity" : C, "location" : Y] -> how many OBJECT_CATEGORIES[C] are in the PLACEMENTS_AND_BEACONS[Y]
Q["action" : "count_objects_placement", "entity" : O, "location" : Y] -> how many OBJECT_NAMES[O] are in the PLACEMENTS_AND_BEACONS[Y]


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
    print grammar
