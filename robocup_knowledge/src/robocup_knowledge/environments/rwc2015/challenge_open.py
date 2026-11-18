from robocup_knowledge import knowledge_functions

''' Hardcoded exploration targets '''
exploration_targets=["explore1",
                     "explore2",
                     "explore3",
                     "explore4",
                     "explore5",
                     "explore6",
                     "explore7",
                     "explore8",
                     "explore9",
                     "explore10",
                     "explore11"
                     ]

initial_pose = "initial_pose_door_B"
operator_waypoint_id = "operator_pose"
exit_waypoint_id = "exit"

# Human robot interactions
# object_options = ['coke','pringles','choco_sticks','beer','juice','tea','coffee']
speech_spec = '((Can you give me (a|an) (item|object) from the <location>) | (Bring me (a|an) (item|object) from the <location>))'


####################################### OLD ###################################
# cabinet = "right_bookcase"
# table1  = "dinnertable"
# table2  = "bookcase"
# table3  = "counter"
# object_shelves=["bookcase","bookcase","bookcase"]

# objects = ["coke", "meadow_milk", "pringles","oblates","chocosticks","cup"]

# #spec find a person and talk with
# spec = "(<object1>|<object1> <object2>|<object1> <object2> <object3>)"

# choices = {'location':[table1, table2, table3],
# 'object1':objects,
# 'object2':objects,
# 'object3':objects}
