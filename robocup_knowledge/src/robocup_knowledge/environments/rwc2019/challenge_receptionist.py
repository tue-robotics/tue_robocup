from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

operator_name = "john"

#TODO
starting_point = "receptionist_start"

#TODO
waypoint_door = {'id': 'look_at_entry_door_wp', 'radius': 0.5}  # Somewhere facing the door to where ppl enter

#TODO:Check
waypoint_livingroom = {'id': 'living_room', 'radius': 0.5}  # From where to find John

#TODO:Check
sitting_room = 'living_room'  # Where people will be guided to sit and the robot will find a seat

#TODO:Check
seats = ['couch', 'armchair']  # List of seats to check for empty-ness. These need an 'on_top_of' volume for the person to sit in
