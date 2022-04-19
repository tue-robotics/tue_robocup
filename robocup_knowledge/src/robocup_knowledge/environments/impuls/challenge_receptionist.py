from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

operator_name = "john"

starting_point = "initial_pose"

waypoint_door = {'id': 'entry_door', 'radius': 0.5}

waypoint_livingroom = {'id': 'livingroom', 'radius': 0.5}

sitting_room = 'living_room'  # Where people will be guided to sit and the robot will find a seat

seats = ['couch_long', 'couch_short']
