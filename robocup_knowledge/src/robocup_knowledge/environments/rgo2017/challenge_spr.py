grammar = '''
Q['answer': "Magdeburg"] -> what city are we in
Q['answer': "Tech United"] -> what is the name of your team
Q['answer': "Chewbacca"] -> name the big hairy creature in star wars
Q['answer': "Isaac Asimov"] -> who wrote the three laws of robotics
Q['answer': "The Jetsons"] -> from what series do you know rosie the robot
Q['answer': "The Flintstones"] -> from what series do you know the baby bam bam
Q['answer': "Neo"] -> who is the main charcater of the matrix
Q['answer': "Peper and HSR"] -> name the two robocupathome standart platforms
Q['answer': "In my SSD"] -> where do you store your memories
Q['answer': "In Eindhoven The Netherlands"] -> where is your team located

WHATWHICH -> what | which

Q['answer': "basket"] -> WHATWHICH is the biggest object
Q['answer': "egg"] -> WHATWHICH is the smallest object
Q['answer': "pringles"] -> WHATWHICH is the biggest food
Q['answer': "egg"] -> WHATWHICH is the smallest food
Q['answer': "basket"] -> WHATWHICH is the biggest container
Q['answer': "coffecup"] -> WHATWHICH is the smallest container
Q['answer': "water"] -> WHATWHICH is the biggest drink
Q['answer': "orange drink"] -> WHATWHICH is the smallest drink
Q['answer': "paper"] -> WHATWHICH is the biggest cleaning stuff
Q['answer': "sponge"] -> WHATWHICH is the smallest cleaning stuff
Q['answer': "knife"] -> WHATWHICH is the biggest cutlery
Q['answer': "fork"] -> WHATWHICH is the smallest cutlery

Q['answer': "the bedroom has two doors"] -> how many doors has the bedroom
Q['answer': "the dining room has two doors"] -> how many doors has the dining room
Q['answer': "in the living room there are no doors"] -> how many doors has the living room
Q['answer': "in the kitchen there are no doors"] -> how many doors has the kitchen

# Counting People

CP[{"action" : "count", "entity" : P}] -> COUNT PEOPLE[P] are in the crowd | tell me COUNT PEOPLE[P] in the crowd

# Arena questions

APLACEM[{"action" : "a_find", "entity" : AP}] -> SEARCH the PLACEMENT[AP]
ABEACON[{"action" : "a_find", "entity" : AB}] -> SEARCH the BEACON[AB]
APLROOM[{"action" : "a_count", "entity" : Y, "location" : R}] -> how many PLACEMENT[Y] are in the ROOM[R]
ABEROOM[{"action" : "a_count", "entity" : Z, "location" : R}] -> how many BEACON[Z] are in the ROOM[R]

COUNT -> how many | the number of

SEARCH -> where is | in WHATWHICH room is

# Object questions

OFIND[{"action" : "o_find", "entity" : O}] -> where can i find a OBJECT[O]
CFIND[{"action" : "o_find", "entity" : C}] -> where can i find a CATEGORY[C]

# - - - - - - - - - - - - - - - - - - - - - - - - -
# D A T A
# - - - - - - - - - - - - - - - - - - - - - - - - -

# People

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

PLACEMENT['bookshelf'] -> bookshelf | bookshelfs
PLACEMENT['couch_table'] -> couch table | couch tables
PLACEMENT['side_table'] -> side table | side_table
PLACEMENT['kitchencounter'] -> kitchen counter | kitchen counters
PLACEMENT['stove'] -> stove | stoves
PLACEMENT['desk'] -> desk |desks
PLACEMENT['bar'] -> bar | bars
PLACEMENT['closet'] -> closet | closets
PLACEMENT['dinner_table'] -> dinner table | dinner tables
PLACEMENT['cabinet'] -> cabinet | cabinets

BEACON['tv_stand'] -> tv stand | tv stands
BEACON['bed'] -> bed | beds
BEACON['sofa'] -> sofa | sofas
BEACON['cupboard'] -> cupboard | cupboards
BEACON['sink'] -> sink | sinks
BEACON['counter'] -> counter | counters

ROOM['dining'] -> dining room
ROOM['living'] -> living room
ROOM['kitchen'] -> kitchen
ROOM['bedroom'] -> bedroom

OBJECT['apple'] -> apple
OBJECT['bread'] -> bread
OBJECT['cereals'] -> cereals
OBJECT['cornflakes'] -> cornflakes
OBJECT['crackers'] -> crackers
OBJECT['lemon'] -> lemon
OBJECT['noodles'] -> noodles
OBJECT['paprika'] -> paprika
OBJECT['peas'] -> peas
OBJECT['pepper'] -> pepper
OBJECT['potato'] -> potato
OBJECT['potato_soup'] -> potato soup
OBJECT['salt'] -> salt
OBJECT['tomato_pasta'] -> tomato pasta
OBJECT['bag'] -> bag
OBJECT['basket'] -> basket
OBJECT['coffecup'] -> coffecup
OBJECT['plate'] -> plate
OBJECT['red_bowl'] -> red bowl
OBJECT['white_bowl'] -> white bowl
OBJECT['banana_milk'] -> banana milk
OBJECT['cappucino'] -> cappucino
OBJECT['coke'] -> coke
OBJECT['orange_drink'] -> orange drink
OBJECT['water'] -> water
OBJECT['chocolate_cookies'] -> chocolate cookies
OBJECT['egg'] -> egg
OBJECT['party_cracker'] -> party cracker
OBJECT['pringles'] -> pringles
OBJECT['cloth'] -> cloth
OBJECT['paper'] -> paper
OBJECT['sponge'] -> sponge
OBJECT['towel'] -> towel
OBJECT['fork'] -> fork
OBJECT['spoon'] -> spoon
OBJECT['knife'] -> knife

CATEGORY['food'] -> food | foods
CATEGORY['container'] -> container | containers
CATEGORY['drink'] -> drink | drinks
CATEGORY['cleaning_stuff'] -> cleaning stuff | cleaning stuffs
CATEGORY['cutlery'] -> cutlery | cutleries

T[X] -> Q[X] | CP[X] | APLACEM[X] | ABEACON[X] | APLROOM[X] | ABEROOM[X] | OFIND[X] | CFIND[X]
'''


big_grammar = """
# - - - - - - - - - - - - - - - - - - - - - - - - -
# Crowd questions
# - - - - - - - - - - - - - - - - - - - - - - - - -



# People Positions/Gestures/Genders

PP[{"action" : "c_count", "entity" : X}] -> how many people in the crowd are POSITION[X]
PG[{"action" : "c_count", "entity" : W}] -> how many people in the crowd are GESTURE[W]
PPG[{"action" : "random_gender", "entity" : X}] -> the POSITION[X] person was GENDER | tell me if the POSITION[X] person was a GENDER
PPGG[{"action" : "random_gender", "entity" : X}] -> the POSITION[X] person was GENDER or GENDER | tell me if the POSITION[X] person was a GENDER or GENDER
PGG[{"action" : "random_gender", "entity" : W}] -> tell me if the GESTURE[W] person was a GENDER
PGGG[{"action" : "random_gender", "entity" : W}] -> tell me if the GESTURE[W] person was a GENDER or GENDER

# People Color

PC[{"action" : "c_count", "entity" : L}] -> tell me how many people were wearing COLOR[L]



# - - - - - - - - - - - - - - - - - - - - - - - - -
# Arena questions
# - - - - - - - - - - - - - - - - - - - - - - - - -



	APLACEM[{"action" : "a_find", "entity" : AP}] -> SEARCH the PLACEMENT[AP]
	ABEACON[{"action" : "a_find", "entity" : AB}] -> SEARCH the BEACON[AB]
		AROOM[{"action" : "a_count", "entity" : R}] -> how many doors has the ROOM[R]
	APLROOM[{"action" : "a_count", "entity" : Y, "location" : R}] -> how many PLACEMENT[Y] are in the ROOM[R]
	ABEROOM[{"action" : "a_count", "entity" : Z, "location" : R}] -> how many BEACON[AB] are in the ROOM[R]

SEARCH -> where is | in WHATWHICH room is



# - - - - - - - - - - - - - - - - - - - - - - - - -
# Object questions
# - - - - - - - - - - - - - - - - - - - - - - - - -



OFIND[{"action" : "o_find", "entity" : O}] -> where can i find a OBJECT[O]
CFIND[{"action" : "o_find", "entity" : C}] -> where can i find a CATEGORY[C]
CCOUNT[{"action" : "o_count", "entity" : C}] -> how many CATEGORY[C] there are

OCOLOR[{"action" : "find_color", "entity" : O}] -> whats the colour of the OBJECT[O]
OTYPE[{"action" : "return_category", "entity" : O}] -> to WHATWHICH category belong the OBJECT[O]
OCAT[{"action" : "compare_category", "entity_a" : O, "entity_b" : A}] -> do the OBJECT[O] and OBJECT[A] belong to the same category

CATPLACE[{"action" : "o_count", "entity" : C, "location" : PL}] -> how many CATEGORY[C] are in the PLACEMENT[PL]
OBJPLACE[{"action" : "o_count", "entity" : O, "location" : PL}] -> how many OBJECT[O] are in the PLACEMENT[PL]
CATLOC[{"action" : "category_at_loc", "location" : PL}] -> what objects are stored in the PLACEMENT[PL]

	CATSIZE[{"action" : "hardcoded"}] -> WHATWHICH is the ADJA CATEGORY[C]
	OBJSIZE[{"action" : "hardcoded"}] -> WHATWHICH is the ADJA object

OBJCOMP[{"action" : "compare", "entity_a" : O, "entity_b" : A}] -> between the OBJECT[O] and OBJECT[A] WHATWHICH one is ADJR


ADJA -> smallest | biggest
ADJR -> smaller | bigger




# - - - - - - - - - - - - - - - - - - - - - - - - -
# D A T A
# - - - - - - - - - - - - - - - - - - - - - - - - -

POSITION['standing'] -> standing
POSITION['sitting'] -> sitting
POSITION['lying'] -> lying

GENDER['male'] -> male
GENDER['female'] -> female
GENDER['man'] -> man
GENDER['woman'] -> woman
GENDER['boy'] -> boy
GENDER['girl'] -> girl

GESTURE['waving'] -> waving
GESTURE['rise_l_arm'] -> rising left arm
GESTURE['rise_r_arm'] -> rising rigth arm
GESTURE['left'] -> pointing left
GESTURE['right'] -> pointing right

COLOR['red'] -> red
COLOR['blue'] -> blue
COLOR['white'] -> white
COLOR['black'] -> black
COLOR['green'] -> green
COLOR['yellow'] -> yellow

"""
