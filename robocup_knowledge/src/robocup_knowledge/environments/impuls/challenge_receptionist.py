from robocup_knowledge import knowledge_functions

operator_name = "john"

operator_drink = "beer"

default_guest_1_name = "adel"
default_guest_1_drink = "cola"

default_guest_2_name = "morgan"
default_guest_2_drink = "orange_juice"


starting_point = "initial_pose"

waypoint_door = {'id': 'entry_door', 'radius': 0.5}

waypoint_livingroom = {'id': 'living_room', 'radius': 0.5}

sitting_room = 'living_room'  # Where people will be guided to sit and the robot will find a seat

# seats = {'couch_long': ['on_top_of'], 'couch_short': ['on_top_of']}
seats = {'couch_long': ['on_top_of_couch_long_left','on_top_of_couch_long_right'], 'couch_short': ['on_top_of_couch_short_left','on_top_of_couch_short_right']}

