# SPR KNOWLEDGE FILE RWC2018

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
Q["action" : "answer", "solution": "I know that Justin Trudeau is very handsome"] -> who is the most handsome person in canada
Q["action" : "answer", "solution": "Canada spans almost 10 million square kilometres and comprises 6 time zones"] -> how many time zones are there in canada
Q["action" : "answer", "solution": "Yonge Street in Ontario is the longest street in the world"] -> what is the longest street in the world
Q["action" : "answer", "solution": "Yonge street is almost 2000 kilometres, starting at Lake Ontario and running north to the Minnesota border"] -> how long is yonge street in ontario
Q["action" : "answer", "solution": "The bear cub was named Winnipeg. It inspired the stories of Winnie the Pooh"] -> what is the name of the bear cub exported from canada to the london zoo in 1915
Q["action" : "answer", "solution": "It was developed in Ontario at Research In Motions Waterloo offices"] -> where was the blackberry smartphone developed
Q["action" : "answer", "solution": "The Big Nickel in Sudbury Ontario. It is nine meters in diameter"] -> what is the worlds largest coin
Q["action" : "answer", "solution": "The first time that the USA invaded Canada was in 1775"] -> in what year was canada invaded by the usa for the first time
Q["action" : "answer", "solution": "The USA invaded Canada a second time in 1812"] -> what year was canada invaded by the usa for the second time
Q["action" : "answer", "solution": "Canada does! With 14 Golds at the 2010 Vancouver Winter Olympics"] -> what country holds the record for the most gold medals at the winter olympics
Q["action" : "answer", "solution": "Sandy Gardiner a journalist of the Ottawa Journal"] -> who coined the term beatlemania
Q["action" : "answer", "solution": "French explorers misunderstood the local native word Kanata which means village"] -> why is canada named canada
Q["action" : "answer", "solution": "The Mounted Police was formed in 1873"] -> when was the mounted police formed
Q["action" : "answer", "solution": "In 1920, when The Mounted Police merged with the Dominion Police"] -> when was the royal canadian mounted police formed
Q["action" : "answer", "solution": "Today the RCMP has close to 30 thousand members"] -> how big is the rcmp
Q["action" : "answer", "solution": "Montreal is often called the City of Saints or the City of a Hundred Bell Towers"] -> what else is montreal called
Q["action" : "answer", "solution": "The Hotel de Glace is in Quebec"] -> where is the hotel de glace located
Q["action" : "answer", "solution": "The Hotel de Glace requires about 400 tons of ice"] -> how many tons of ice are required to build the hotel de glace
Q["action" : "answer", "solution": "Every year 12 thousand tons of snow are used for The Hotel de Glace"] -> how many tons of snow are required to build the hotel de glace
Q["action" : "answer", "solution": "No. Every summer it melts away only to be rebuilt the following winter"] -> can i visit the hotel de glace in summer
Q["action" : "answer", "solution": "Canadas only desert is British Columbia"] -> where is canadas only desert
Q["action" : "answer", "solution": "The British Columbia desert is only 15 miles long"] -> how big is canadas only desert
Q["action" : "answer", "solution": "Leonard Cohen Keanu Reeves and Jim Carrey"] -> name three famous male canadians
Q["action" : "answer", "solution": "Celine Dion Pamela Anderson and Avril Lavigne"] -> name three famous female canadians
Q["action" : "answer", "solution": "Comic Sans is based on Dave Gibbons lettering in the Watchmen comic books"] -> what is the origin of the comic sans font
Q["action" : "answer", "solution": "The smallest robot possible is called a nanobot"] -> what is a nanobot
Q["action" : "answer", "solution": "A nanobot can be less than one thousandth of a millimeter"] -> how small can a nanobot be
Q["action" : "answer", "solution": "The Academy thought that Tron cheated by using computers"] -> why wasnt tron nominated for an award by the motion picture academy
Q["action" : "answer", "solution": "The IBM 305 RAMAC"] -> which was the first computer with a hard disk drive
Q["action" : "answer", "solution": "The IBM 305 RAMAC was launched in 1956"] -> when was the first computer with a hard disk drive launched
Q["action" : "answer", "solution": "The IBM 305 RAMAC hard disk weighed over a ton and stored 5 megabytes of data"] -> how big was the first hard disk drive
Q["action" : "answer", "solution": "CAPTCHA is an acronym for Completely Automated Public Turing test to tell Computers and Humans Apart"] -> what does captcha stand for
Q["action" : "answer", "solution": "The first actual computer bug was a dead moth stuck in a Harvard Mark 2"] -> what was the first computer bug
Q["action" : "answer", "solution": "There are four robots on Mars Sojourner Spirit Opportunity and Curiosity. Three more crashed on landing"] -> name all of the robots on mars
Q["action" : "answer", "solution": "Professor Kevin Warwick uses chips in his arm to operate doors a robotic hand and a wheelchair"] -> who is the worlds first android
Q["action" : "answer", "solution": "A robot sketch made by Leonardo DaVinci"] -> what is a mechanical knight
Q["action" : "answer", "solution": "Some people think it was IBM Watson but it was Eugene a computer designed at Englands University of Reading"] -> what was the first computer to pass the turing test
Q["action" : "answer", "solution": "Moravecs paradox states that a computer can crunch numbers like Bernoulli but lacks a toddlers motor skills"] -> what does moravecs paradox state
Q["action" : "answer", "solution": "It is when you need to load an AI with enough knowledge to start learning"] -> what is the ai knowledge engineering bottleneck
Q["action" : "answer", "solution": "I dont know. He should worry more about the peoples impact on humanity"] -> why is elon musk worried about ais impact on humanity
Q["action" : "answer", "solution": "No. Humans are the real threat to humanity"] -> do you think robots are a threat to humanity
Q["action" : "answer", "solution": "A chatbot is an AI you put in customer service to avoid paying salaries"] -> what is a chatbot
Q["action" : "answer", "solution": "Yes. Car accidents are product of human misconduct"] -> are self driving cars safe
Q["action" : "answer", "solution": "Grace Hoper. She wrote it in her spare time"] -> who invented the compiler
Q["action" : "answer", "solution": "C was invented by Dennis MacAlistair Ritchie"] -> who created the c programming language
Q["action" : "answer", "solution": "Python was invented by Guido van Rossum"] -> who created the python programming language
Q["action" : "answer", "solution": "Sure. I have never seen him drink water"] -> is mark zuckerberg a robot
Q["action" : "answer", "solution": "My lord and master Steve Wozniak"] -> who is the inventor of the apple one microcomputer
Q["action" : "answer", "solution": "Ada Lovelace"] -> who is considered to be the first computer programmer
Q["action" : "answer", "solution": "Adobe Wan Kenobi"] -> which program do jedi use to open pdf files

WHATWHICH -> what | which

Q["action" : "answer", "solution": "tray"] -> WHATWHICH is the biggest object
Q["action" : "answer", "solution": "scrubby"] -> WHATWHICH is the smallest object
Q["action" : "answer", "solution": "pringles"] -> WHATWHICH is the biggest food
Q["action" : "answer", "solution": "egg"] -> WHATWHICH is the smallest food
Q["action" : "answer", "solution": "orange and grape juice"] -> WHATWHICH is the biggest drink
Q["action" : "answer", "solution": "coke and sprite"] -> WHATWHICH is the smallest drink

Q["action" : "answer", "solution": "in the dining room there are no doors"] -> how many doors has the diningroom
Q["action" : "answer", "solution": "the living room has two doors"] -> how many doors has the livingroom
Q["action" : "answer", "solution": "the bedroom has two doors"] -> how many doors has the bedroom
Q["action" : "answer", "solution": "the kitchen has only one door"] -> how many doors has the kitchen
Q["action" : "answer", "solution": "the corridor has only one door"] -> how many doors has the corridor
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

NQ["action" : "count_pos", "entity" : X] -> how many people in the crowd are POSITION[X]
NQ["action" : "count_ges", "entity" : W] -> how many people in the crowd are GESTURE[W]
NQ["action" : "count_col", "entity" : L] -> tell me how many people were wearing COLOR[L]

NQ["action" : "gender_pos", "entity" : X] -> the POSITION[X] person was GENDER | tell me if the POSITION[X] person was a GENDER | the POSITION[X] person was GENDER or GENDER | tell me if the POSITION[X] person was a GENDER or GENDER
NQ["action" : "gender_ges", "entity" : W] -> tell me if the GESTURE[W] person was a GENDER | tell me if the GESTURE[W] person was a GENDER or GENDER
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
SB -> smaller | bigger
LH -> lighter | heavier


Q["action" : "find_object", "entity" : O] -> where can i find DET OBJECT_NAMES[O]
Q["action" : "find_category", "entity" : C] -> where can i find DET OBJECT_CATEGORIES[C]

Q["action" : "return_category", "entity" : O] -> to WHATWHICH category belong the OBJECT_NAMES[O]
Q["action" : "return_color", "entity" : O] -> whats the color of the OBJECT_NAMES[O]

Q["action" : "compare_category", "entity_a" : O, "entity_b" : A] -> do DET OBJECT_NAMES[O] and DET OBJECT_NAMES[A] belong to the same category
Q["action" : "compare_sizes", "entity_a" : O, "entity_b" : A] -> between DET OBJECT_NAMES[O] and DET OBJECT_NAMES[A] which one is SB
Q["action" : "compare_weight", "entity_a" : O, "entity_b" : A] -> between DET OBJECT_NAMES[O] and DET OBJECT_NAMES[A] which one is LH

Q["action" : "count_object_cat", "entity" : C] -> how many OBJECT_CATEGORIES[C] there are

NQ["action" : "count_object_loc", "location" : Y] -> how many objects are in the PLACEMENTS_AND_BEACONS[Y] | what objects are stored in the PLACEMENTS_AND_BEACONS[Y]
NQ["action" : "count_object_cat_loc", "entity" : C, "location" : Y] -> how many OBJECT_CATEGORIES[C] are in the PLACEMENTS_AND_BEACONS[Y]
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
POSITION['lying'] -> laying

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
