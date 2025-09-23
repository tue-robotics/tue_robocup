# GERMAN OPEN 2015
from robocup_knowledge import knowledge_functions

#locations:
starting_point = "initial_pose_door_A"
meeting_point = "gpsr_meeting_point"

#data for speech recognition
objects_known = ["ice-tea", "oblates", "bubblegum", "candle", "cup", "beer", "wd40", "filler", "chocolate cereals", "coke",
                 "chocosticks", "yellow candle", "noodles", "coffee", "cranberry cereals", "muesli cereals", "bubblemint",
                 "deodorant", "brush", "cigarettes", "red bull", "meadow milk", "mints", "pringles", "fresh milk", "coffeepads",
                 "peanut"]

object_types_plural = ["decoration", "food", "drinks", "tools", "leisure"]
object_types_singular = ["drink", "snack", "bowl", "tray", "tool", "leisure"]
object_types = object_types_plural + object_types_singular

location_placement =   ['left bedside table', 'dinnertable', 'cupboard',
                        'bartable', 'stove', 'counter', 'left bookcase', 'fridge',
                        'bed', 'sink', 'desk', 'small table',
                        'kitchencounter', 'cabinet', 'couchtable', 'right bedside table', 'right bookcase']

##NEW, location_types:
location_types = ['', 'shelf', 'appliance', 'seat', 'table', 'utility']

rooms = ["kitchen", "livingroom", "hall", "bedroom", "office"]
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
'3_name_time_date':['your name', 'the name of your team', 'the time', 'what time is it', 'tell the date', 'what day is today', 'what day is today', 'what day is tomorrow',  'tell the day of the month', ' tell the day of the week']}
