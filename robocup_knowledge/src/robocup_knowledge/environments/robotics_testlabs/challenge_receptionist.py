from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

operator_name = "john"

starting_point = "initial_pose"

waypoint_door = {'id': 'entry_door', 'radius': 0.5}

waypoint_livingroom = {'id': 'livingroom', 'radius': 0.5}

default_target_radius = 0.2

drink_names = [obj['name'] for obj in common.objects if obj['category'] == 'drink']

drink_spec = "T[O] -> OPTIONS[O]\n\n"
for dn in drink_names:
    drink_spec += "OPTIONS['{drink}'] -> {drink}\n".format(drink=dn)
