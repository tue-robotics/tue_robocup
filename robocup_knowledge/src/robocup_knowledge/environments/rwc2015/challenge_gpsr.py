# RWC2015, to be adjusted when in hall.


from robocup_knowledge import knowledge_loader
challenge_speech_recognition_data = knowledge_loader.load_knowledge("challenge_speech_recognition")

spec_questions = challenge_speech_recognition_data.spec 
choices_questions = challenge_speech_recognition_data.choices 


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

persons_women = ["Anna","Beth","Carmen","Jennifer","Jessica","Kimberly","Kristina","Laura","Mary","Sarah"]
persons_men = ["Alfred","Charles","Daniel","James","John","Luis","Paul","Richard","Robert","Steve"]

#spec_get_and_deliver
spec_get_deliver = "(<2_vb_take> the <2_object> from the <1_location> and <3_vb_deliver> it to (<3_person_me>|(the <3_place_location>)|(<3_person> (at|in|(which is in)) the <3_room>)))"

#spec go to room and search for an object
spec_goroom_findobj = "(<1_vb_goto> the <1_room> and <2_vb_find> the <2_object>)"

#spec find a person and talk with
spec_findperson_talk = "(<2_vb_find> a person in the <1_room> and ((answer a <3_question>)|(<3_vb_speak> <3_name_time_date>)))"

spec = "("+spec_get_deliver+"|"+spec_goroom_findobj+"|"+spec_findperson_talk+")"
#spec = spec_findperson_talk

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

