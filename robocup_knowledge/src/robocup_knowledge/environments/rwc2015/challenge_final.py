initial_pose_amigo = "initial_pose_amigo"
operator_id = "operator"

## Amigo speech:

# First asking action
grasp_locations = ['kitchentable', 'kitchencounter', 'cupboard', 'bar', 'couchtable', 'dinnertable', 'desk', 'bookcase', 'hallwaytable']
spec1_amigo_task = "((<name> (needs|wants) your help) | ((go|get) back to <name>) | ((bring|get|fetch) me the object (on|from) the <grasp_location>) | (<time>))"
choices1_amigo_task = {'name':['Luis','Erik','Rokus','Sjoerd','operator','the operator'], 'person_location':grasp_locations, 'grasp_location':grasp_locations, 'time':['How much time do we have left']}

# Second asking action
spec2_amigo_task_followup = "((near|at) the <location>)"
choices2_amigo_task_followup = {'location':grasp_locations}

