choice_answer_mapping = {

	# Predefined questions:

	"What city are we in?": "Magdeburg",
	"What is the name of your team?": "Tech United",
	"Name the big hairy creature in Star Wars.": "Chewbacca",
	"Who wrote the three laws of robotics?": "Isaac Asimov",
	"From what series do you know Rosie the robot?": "The Jetsons",
	"From what series do you know the baby Bam Bam": "The Flintstones",
	"Who is the main charcater of The Matrix": "Neo",
	"Name the two RoboCup@Home standart platforms.": "Peper and HSR",
	"Where do you store your memories": "In my SSD",
	"Where is your team located?": "In Eindhoven The Netherlands",

	# Crowd questions:

	# "How many PEOPLE are in the crowd?" | "Tell me the number of PEOPLE in the crowd"
	# "How many people in the crowd are POSITION | GESTURE?"
	# "The POSITION person was GENPPL?"
	# "Tell me if the (POSPRS	| GESTURE) person was a GENPPL?"
	# "Tell me how many people were wearing COLOR"
	
	"How many males are there in the crowd?": "Crowd_males",
	"How many females are there in the crowd?": "Crowd_females",
	"What is the size of the crowd?": "Crowd_size",
	"How many children are there in the crowd?": "Crowd_children",

	# Arena questions:

	# "Where is the {placement}?"
	# "Where is the {beacon}?"
	# "In which room is the {placement}?"
	# "In which room is the {beacon}?"
	# "How many doors has the {room}?"
	# "How many ({placement} | {beacon}) are in the {room}?"

	# Object questions:

	"Which is the biggest object?": "",
	"Which is the smallest object?": "",

	"Which is the biggest food?": "",
	"Which is the smallest food?": "",

	"Which is the biggest container?": "",
	"Which is the smallest container?": "",

	"Which is the biggest drink?": "",
	"Which is the smallest drink?": "",

	"Which is the biggest cleaning stuff?": "",
	"Which is the smallest cleaning stuff?": "",

	"Which is the biggest cutlery?": "",
	"Which is the smallest cutlery?": ""

	# "Where can I find a {object}?"
	# "How many {category} there are?"
	# "Whats the colour of the {kobject}?"
	# "How many ({category} | objects) are in the {placement}?"
	# "What objects are stored in the {placement}?"
	# "Where can I find a ({object} | {category})?"
	# "To which category belong the {object}?"
	# "Do the {object 1} and {object 2} belong to the same category?"
	# "Between the {object 1} and {object 2}, which one is $adjr?"

	# $adja = heaviest | smallest | biggest | lightest
	# $adjr = heavier | smaller | bigger | lighter

}

grammar = """
# - - - - - - - - - - - - - - - - - - - - - - - - -
# Crowd questions
# - - - - - - - - - - - - - - - - - - - - - - - - -



# Counting People

CP[{"action" : "count", "entity" : P}] -> COUNT PEOPLE[P] are in the crowd | tell me COUNT PEOPLE[P] in the crowd

COUNT -> how many | the number of

# People Positions/Gestures/Genders

PP[{"action" : "c_count", "entity" : X}] -> how many people in the crowd are POSITION[X]
PG[{"action" : "c_count", "entity" : W}] -> how many people in the crowd are GESTURE[W]
PPG[{"action" : "return_gender", "entity" : X}] -> the POSITION[X] person was GENDER | tell me if the POSITION[X] person was a GENDER
PPGG[{"action" : "return_gender", "entity" : X}] -> the POSITION[X] person was GENDER or GENDER | tell me if the POSITION[X] person was a GENDER or GENDER
PGG[{"action" : "return_gender", "entity" : W}] -> tell me if the GESTURE[W] person was a GENDER
PGGG[{"action" : "return_gender", "entity" : W}] -> tell me if the GESTURE[W] person was a GENDER or GENDER

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

SEARCH -> where is | in which room is



# - - - - - - - - - - - - - - - - - - - - - - - - -
# Object questions
# - - - - - - - - - - - - - - - - - - - - - - - - -



OFIND[{"action" : "o_find", "entity" : O}] -> where can i find a OBJECT[O]
CFIND[{"action" : "o_find", "entity" : C}] -> where can i find a CATEGORY[C]
CCOUNT[{"action" : "o_count", "entity" : C}] -> how many CATEGORY[C] there are

OCOLOR[{"action" : "find_color", "entity" : O}] -> whats the colour of the OBJECT[O]
OTYPE[{"action" : "return_category", "entity" : O}] -> to which category belong the OBJECT[O]
OCAT[{"action" : "compare_category", "entity_a" : O, "entity_b" : A}] -> do the OBJECT[O] and OBJECT[A] belong to the same category

CATPLACE[{"action" : "o_count", "entity" : C, "location" : PL}] -> how many CATEGORY[C] are in the PLACEMENT[PL]
OBJPLACE[{"action" : "o_count", "entity" : O, "location" : PL}] -> how many OBJECT[O] are in the PLACEMENT[PL]
CATLOC[{"action" : "category_at_loc", "location" : PL}] -> what objects are stored in the PLACEMENT[PL]

CATSIZE[{"action" : "hardcoded"}] -> which is the ADJA CATEGORY[C]
OBJSIZE[{"action" : "hardcoded"}] -> which is the ADJA object

OBJCOMP[{"action" : "compare", "entity_a" : O, "entity_b" : A}] -> between the OBJECT[O] and OBJECT[A] which one is ADJR


ADJA -> smallest | biggest
ADJR -> smaller | bigger




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

# Arena

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

BEACON['tv_stand'] -> tv stand
BEACON['bed'] -> bed
BEACON['sofa'] -> sofa
BEACON['cupboard'] -> cupboard | cupboards
BEACON['sink'] -> sink | sinks
BEACON['counter'] -> counter | counters

ROOM['dining'] -> dining room
ROOM['living'] -> living room
ROOM['kitchen'] -> kitchen
ROOM['bedroom'] -> bedroom

# Object

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

T[X] -> CP[X] | PP[X] | PG[X] | PPG[X] | PPGG[X] | PGG[X] | PGGG[X] | PC[X] | APLACEM[X] | ABEACON[X] | AROOM[X] | APLROOM[X] | ABEROOM[X] | OFIND[X] | CFIND[X] | CCOUNT[X] | OCOLOR[X] | OCAT[X] | CATPLACE[X] | OBJPLACE[X] | CATLOC[X] | OBJCOMP[X]
"""