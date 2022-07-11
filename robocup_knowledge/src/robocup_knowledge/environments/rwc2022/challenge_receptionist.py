from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

operator_name = "john"

operator_drink = "beer"

starting_point = "starting_pose"

waypoint_door = {'id': 'entry_door', 'radius': 0.25}

waypoint_livingroom = {'id': 'living_room', 'radius': 0.5}

sitting_room = 'living_room'  # Where people will be guided to sit and the robot will find a seat

seats = ['couch']
