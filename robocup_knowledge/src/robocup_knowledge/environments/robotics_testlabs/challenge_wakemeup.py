from robocup_knowledge import knowledge_functions

# Tunable params
find_person = {                 # This needs to be updated according to the environment
    'within_range' : 2.0,
    'under_z' : 0.3,
    'min_chull_area' : 0.06,
    'min_exist_prob' : 0.6
}

alarm_wait_time = 5
alarm_duration = 60

get_newspaper_timeout = 3       # This needs to be updated for robocup maybe
give_newspaper_timeout = 30
wakeup_light_color = [1, 1, 1]

handover_joint_config = "sensed_handover"

sensor_range = 2.0

order_confirmation_limit = 3

initial_pose = "initial_pose"

# Knowledge
bed = 'bed'                     # This needs to be updated according to the environment
matress_height = 0.70           #TODO: Set matress height or even better: get from models

bed_nav_goal = {
    'near' : bed,
    'in' : 'bedroom',
    'lookat' : 'bed',
    'at_bedside' : bed,
}

default_milk = "fresh milk"
default_cereal = "coconut_cereals"
default_fruit = "apple"

kitchen_nav_goal = {            # This needs to be updated according to the environment
    "in_front_of":"cabinet",
    "in" : "kitchen",
    "lookat" : "cabinet"
}

generic_items = [ "milk", "cereal", "fruit" ]
allowed_fruits = [ "apple" ]

milk_shelf = "cabinet"          # This needs to be updated according to the environment
cereal_shelf = "cabinet"        # This needs to be updated according to the environment
fruit_shelf = "cabinet"         # This needs to be updated according to the environment

table_nav_goal = {
    "in": "kitchen",
    "near": "kitchen_table",
    "in_front_of": "dinnertable",
    "lookat" : "dinnertable"
}

item_nav_goal = {               # This needs to be updated according to the environment
    "in" : "kitchen",
    "near_milk" : milk_shelf,
    "lookat_milk" : milk_shelf,
    "near_cereal" : cereal_shelf,
    "lookat_cereal" : cereal_shelf,
    "near_fruit" : fruit_shelf,
    "lookat_fruit" : fruit_shelf
}

# milk_nav_goal = {               # This needs to be updated according to the environment
#     "in" : "kitchen",
#     "near" : milk_shelf,
#     "lookat" : milk_shelf
# }

# cereal_nav_goal = {             # This needs to be updated according to the environment
#     "in" : "kitchen",
#     "near" : cereal_shelf,
#     "lookat" : cereal_shelf
# }

# fruit_nav_goal = {              # This needs to be updated according to the environment
#     "in" : "kitchen",
#     "near" : fruit_shelf,
#     "lookat" : fruit_shelf
# }

kitchen_door = "kitchen_door"   # Only needed if we can open a door

# Number of times it asks for the door to be opened and replans to the kitchen
check_door_attempts = 1

dinner_table = "dinner_table"


