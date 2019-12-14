from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

operator_name = "john"

starting_point = "receptionist_start"

waypoint_door = {'id': 'look_at_entry_door_wp', 'radius': 0.5}  # Somewhere facing the door to where ppl enter

waypoint_livingroom = {'id': 'living_room', 'radius': 0.5}  # From where to find John

sitting_room = 'living_room'  # Where people will be guided to sit and the robot will find a seat

seats = ['couch', 'armchair']  # List of seats to check for empty-ness. These need an 'on_top_of' volume for the person to sit in
