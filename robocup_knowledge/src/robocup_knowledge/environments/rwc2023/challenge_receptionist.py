from robocup_knowledge import knowledge_loader

common = knowledge_loader.load_knowledge("common")

operator_name = "john"

operator_drink = "milk"

default_guest_1_name = "adel"
default_guest_1_drink = "cola"

default_guest_2_name = "morgan"
default_guest_2_drink = "orange_juice"

starting_point = "initial_pose_receptionist"

waypoint_door = {'id': 'entrance', 'radius': 0.2}

waypoint_livingroom = {"id": "receptionist_living_room", "radius": 0.2}

sitting_room = "living_room"  # Where people will be guided to sit and the robot will find a seat

seats = {"sofa": ["on_top_of_sofa_left", "on_top_of_sofa_right"], "chair1": ["on_top_of"], "chair2": ["on_top_of"]}
