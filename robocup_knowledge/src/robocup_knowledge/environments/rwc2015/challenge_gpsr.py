# RWC2015
from robocup_knowledge import knowledge_functions

from robocup_knowledge import knowledge_loader
challenge_speech_recognition_data = knowledge_loader.load_knowledge("challenge_speech_recognition")

spec_questions = challenge_speech_recognition_data.spec
choices_questions = challenge_speech_recognition_data.choices

#locations: # TO BE DEFINED IN model.yaml!
starting_point = "initial_pose_door_A"
meeting_point = "gpsr_meeting_point"
gpsr_exit = "exit_door_B1"

# locations inside room used for finding object in room.
rooms_detailed = {  'kitchen':['kitchentable','kitchencounter', 'cupboard'], # fridge and trashbin are no manipulation locations
                    'livingroom':['bar','couchtable','dinnertable','sofa'], # dinnertable and sofa have both two spots
                    'bedroom':['left_bedside_table','right_bedside_table','desk','bed'],# bed has two spots
                    'hallway':['hallwaytable', 'bookcase/shelf1','bookcase/shelf2','bookcase/shelf3','bookcase/shelf4','bookcase/shelf5','bookcase/shelf6','bookcase/shelf7','bookcase/shelf8']} #,'bookcase']} # both locations have multiple spots.

objects_known_recognize = ['pure_milk', 'orange_juice', 'sponge', 'papaya_milk', 'apple', 'tomato_chips', 'lemon', 'toothpaste',
                 'chocolates', 'bowl', 'beer', 'toilet_paper', 'soap', 'plate', 'pear', 'lotion', 'water', 'cloth',
                 'green_tea', 'gram_soup', 'bubble_gum', 'bean_sauce', 'barbecue_chips', 'tray', 'coconut_cereals',
                 'egg_stars', 'honey_chips', 'coco_balls', 'biscuits']

###############################################
#### GPSR KNOWLEDGE FOR ASKING FOR ACTION: ####
###############################################

#data for speech recognition

objects_known = ['pure milk', 'orange juice', 'sponge', 'papaya milk', 'apple', 'tomato chips', 'lemon', 'toothpaste',
                 'chocolates', 'bowl', 'beer', 'toilet paper', 'soap', 'plate', 'pear', 'lotion', 'water', 'cloth',
                 'green tea', 'gram soup', 'bubble gum', 'bean sauce', 'barbecue chips', 'tray', 'coconut cereals',
                 'egg stars', 'honey chips', 'coco balls', 'biscuits']

location_placement = ['kitchentable', 'kitchencounter', 'cupboard', 'bar', 'couchtable', 'dinnertable', 'sofa',
                      'left bedside table', 'right bedside table', 'desk', 'bed', 'bookcase', 'hallwaytable']

#location_placement = ['kitchentable','dinnertable']

rooms = ["kitchen", "hallway", "livingroom", "bedroom"]
#rooms = ["hallway"]

persons_women = ["Alex","Angel","Eve","Jamie","Jane","Liza","Melissa","Tracy","Robin","Sophia"]
persons_men = ["Alex","Angel","Edward","Homer","Jamie","John","Kevin","Kurt","Tracy","Robin"]

#spec_get_and_deliver
spec_get_deliver = "(<2_vb_take> the <2_object> from the <1_location> and <3_vb_deliver> it to (<3_person_me>|(the <3_place_location>)|(<3_person> (at|in|(which is in)) the <3_room>)))"
#spec_get_deliver = "(<2_vb_take> the <2_object> from the <1_location> and <3_vb_deliver> it to the <3_place_location>)"

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

