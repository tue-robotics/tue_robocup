
#locations:
starting_point = "initial_pose"
meeting_point = "gpsr_meeting_point"
gpsr_exit = "exit"

# locations inside room used for finding object in room.
rooms_detailed = {  'kitchen':['chair','cabinet'],
                    'hallway':['hallway_couch'],
                    'living_room':['couchtable','dinnertable','corridor_table','bookcase'],
                    'bedroom':['bed','bed_cabinet'],
                    'workshop':['operator_table','workbench']}

###############################################
#### GPSR KNOWLEDGE FOR ASKING FOR ACTION: ####
###############################################

#data for speech recognition
objects_known = ['beer','coke','beer','coke','fanta','ice_tea','tea','coffee_pads','deodorant','frutas','sprite','teddy']

location_placement = ['chair','cabinet', 'hallway couch','couchtable','dinnertable','corridor table','bookcase','bed','bed cabinet','operator table','workbench']

rooms = ["kitchen", "hallway", "living room", "bedroom", "workshop"]
#rooms = ["living room"]
persons_women = ["Anna","Beth","Carmen","Jennifer","Jessica","Kimberly","Kristina","Laura","Mary","Sarah"]
persons_men = ["Alfred","Charles","Daniel","James","John","Luis","Paul","Richard","Robert","Steve"]

#spec_get_and_deliver
spec_get_deliver = "(<2_vb_take> the <2_object> from the <1_location> and <3_vb_deliver> it to (<3_person_me>|(the <3_place_location>)|(<3_person> (at|in|(which is in)) the <3_room>)))"

#spec go to room and search for an object
spec_goroom_findobj = "(<1_vb_goto> the <1_room> and <2_vb_find> the <2_object>)"

#spec find a person and talk with
spec_findperson_talk = "(<2_vb_find> a person in the <1_room> and ((answer a <3_question>)|(<3_vb_speak> <3_name_time_date>)))"

spec = "("+spec_get_deliver+"|"+spec_goroom_findobj+"|"+spec_findperson_talk+")"
#spec = spec_get_deliver

choices = {'1_location':location_placement,
'2_vb_take':['take', 'grasp', 'get'],   
'2_object':objects_known,
'3_vb_deliver': ['bring', 'carry', 'deliver', 'take'],
'3_person_me':['me'],
'3_person':persons_men+persons_women,
'3_place_location':location_placement,
'3_room':rooms,

'1_vb_goto':['go to', 'navigate to', 'reach', 'get into'],
'1_room':rooms,
'2_vb_find':['find', 'look for'],
'3_question':['question'],
'3_vb_speak':['tell', 'say', 'speak'],
'3_name_time_date':['your name', 'the name of your team', 'the time', 'what time is it', 'what time it is', 'the date', 'what day is today', 'what day is tomorrow',  'the day of the month', 'the day of the week']}


##########################################
#### KNOWLEDGE FOR ASKING A QUESTION: ####
##########################################

from datetime import datetime
choice_answer_mapping = {
        "Which American state is nearest to the former Soviet Union ":"ALASKA",
        "How many tentacles does a squid have ":"TEN",
        "What is converted into alcohol during brewing ":"SUGAR",
        "Which river forms the eastern section of the border between England and Scotland ":"TWEED",
        "In what year was Prince Andrew born ":"1960  19th February ",
        "Name the two families in Romeo and Juliet ":"MONTAGUE & CAPULET",
        "If cats are feline  what are sheep ":"OVINE",
        "For which fruit is the US state of Georgia famous ":"PEACH",
        "Which is the financial centre and main city of Switzerland ":"ZURICH",
        "Which TV programme s theme tune was called Hit and Miss ":"JUKE BOX JURY",
        "Which guitarist is known as Slowhand ":"ERIC CLAPTON",
        "What is an infant whale commonly called ":"CALF",
        "What do the British call the vegetables that Americans call zucchini ":"COURGETTES",
        "What is an otter s home called ":"HOLT",
        "How have vegetables been cut which are served Julienne ":"THIN STRIPS or shreds or sliced lengthways ",
        "In Roman mythology  Neptune is the equivalent to which Greek god ":"POSEIDON",
        "Which TV character said   Live long and prosper  ":"MR SPOCK  Star Trek ",
        "In which State would you find the city of Birmingham ":"ALABAMA",
        "Which hills divide England from Scotland ":"CHEVIOTS",
        "What continent has the fewest flowering plants ":"ANTARTICA",
        "What is Canada s national animal ":"BEAVER",
        "What is the alternative common name for a Black Leopard ":"PANTHER",
        "What in Cornwall is the most southerly point of mainland Britain ":"LIZARD POINT",
        "What explorer introduced pigs to North America ":"CRISTOPHER COLUMBUS",
        "What is the world biggest island ":"GREENLAND",
        "what time is it":"It is %s"%datetime.now().strftime("%I %M %p"),
        "what is the capital of germany":"berlin",
        "what is the heaviest animal in the world":"The blue whale",
        "who is the president of america":"barack obama",
        "what is your name":"It's me Amigo",
        "who is the ugliest person in the world":"Rein Appeldoorn",
        "what is your motto":"I am a banana!",
        "which football club is the best":"Ajax",
        "who is the best looking person around here":"Simple question, Erik Geerts",
        "which team member is your favorite operator":"that i cannot choose, but if I have to, it would be my dear Erik",
        "which town has been bombed":"Rotterdam and Schijndel",
        "which person is not able to say yes":"Dirk Holtz",
        "what is the capital of brazil":"brasilia",
        "what is the oldest drug used on earth":"alcohol",
        "in which year was robocup founded":"nineteen ninety seven",
        "how many rings has the olympic flag":"five",
        "what is the worlds most popular green vegetable":"lettuce",
        "which insect has the best eyesight":"dragonfly",
        "who lives in a pineapple under the sea":"spongebob squarepants",
        "what is your teams name":"Tech united eindhoven",
        "what is the answer to the ultimate question about life the universe and everything":"forty two",
        "what is the capital of poland":"warsaw",
        "which country grows the most potatoes":"russia",
        "which country grew the first orange":"china",
        "how many countries are in europe":"fifty",
        "Who is the banana king":"Sjoerd van den Dries",
        "which fish can hold objects in its tail":"sea horse"
}

spec_question = '<question>'
choices_question = {'question': [k for k,v in choice_answer_mapping.iteritems()]}
