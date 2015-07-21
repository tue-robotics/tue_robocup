initial_pose_amigo = "initial_pose_amigo"
operator_id = "operator"

## Amigo speech:

# First asking action
grasp_locations = ['kitchentable', 'kitchencounter', 'cupboard', 'bar', 'couchtable', 'dinnertable', 'desk', 'bookcase', 'hallwaytable']
spec1_amigo_task = "((<name> (needs|wants) your help)| ((bring|get|fetch) me the object on the <location>))"
choices1_amigo_task = {'name':['Luis','Erik','Rokus','Janno'], 'location':grasp_locations}

# Second asking action
spec2_amigo_task_followup = "((near|at) the <location>))"
choices1_amigo_task_followup = {'location':grasp_locations}

