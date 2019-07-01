from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

operator_name = "john"

starting_point = "initial_pose"

waypoint_door = {'id': 'door_opening_right_start', 'radius': 0.5}  # Somewhere facing the door to where ppl enter

waypoint_livingroom = {'id': 'livingroom', 'radius': 0.5}  # From where to find John

sitting_room = 'hallway'  # Where people will be guided to sit and the robot will find a seat

seats = ['hallway_table']  # List of seats to check for empty-ness
