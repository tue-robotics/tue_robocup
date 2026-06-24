from robocup_knowledge import knowledge_functions

initial_pose_amigo = "initial_pose_amigo"
operator_id = "operator"

## Amigo speech:

# First asking action
grasp_locations = ['kitchentable', 'kitchencounter', 'cupboard', 'bar', 'couchtable', 'dinnertable', 'desk', 'bookcase', 'hallwaytable','bed']
# FOR TESTING:
#grasp_locations = ['dinnertable']

person_locations = ['kitchentable', 'kitchencounter', 'cupboard', 'bar', 'dinnertable', 'desk', 'hallwaytable','bed']
# FOR TESTING:
#person_locations = ['bar', 'bed']

spec1_amigo_task = "((<name> (needs|wants) your help) | ((go|get) back to <name>) | ((bring|get|fetch) me the object (on|from) the <grasp_location>) | (<time>))"
# FOR TESTING:
#spec1_amigo_task = "((<name> (needs|wants) your help) | ((go|get) back to <name>) | ((bring|get|fetch) me the object (on|from) the <grasp_location>))"
#spec1_amigo_task = "((bring|get|fetch) me the object (on|from) the <grasp_location>)"

choices1_amigo_task = {'name':['Luis','Erik','Sjoerd','operator','the operator'], 'person_location':person_locations, 'grasp_location':grasp_locations, 'time':['How much time do we have left','How much time do we still have']}

# Second asking action
spec2_amigo_task_followup = "((near|at) the <location>)"
choices2_amigo_task_followup = {'location':person_locations}

