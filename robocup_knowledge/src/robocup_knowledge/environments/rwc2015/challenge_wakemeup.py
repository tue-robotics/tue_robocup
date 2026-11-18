from robocup_knowledge import knowledge_functions

# Tunable params
find_person = {
    'within_range' : 2.0,
    'under_z' : 0.5,
    'min_chull_area' : 0.06,
    'min_exist_prob' : 0.6
}

alarm_wait_time = 5
alarm_duration = 50

get_newspaper_timeout = 5
give_newspaper_timeout = 30
wakeup_light_color = [1, 1, 1]

handover_joint_config = "sensed_handover"

sensor_range = 2.0

# Knowledge
order_confirmation_limit = 3

initial_pose = "initial_pose_door_A"

bed = 'bed'
mattress_height = 0.51

bed_nav_goal = {
    'near' : bed,
    'at_bedside' : bed,
    'in' : 'bedroom', # Maybe define a bedside to get a good vantage point and position for handing over the newspaper
    'lookat' : bed
}

default_milk = "pure_milk"
default_cereal = "coconut_cereals"
default_fruit = "apple"


# TODO: in front of?
kitchen_nav_goal = {
    # "in_front_of":"kitchentable", # This may not be necessary?
    "in" : "kitchen",
    "lookat" : "kitchen"
}

# This also defines the order in which to pick the items!!!
generic_items = [ "milk", "fruit", "cereal"]
# TODO: Pour Cereal

milk_shelf = "kitchentable"
cereal_shelf = "kitchencounter"
fruit_shelf = "kitchencounter"

table_nav_goal = {
    "in": "livingroom",
    "in_front_of": "dinnertable",
    "lookat": "dinnertable"
}

item_nav_goal = {               # This needs to be updated according to the environment
    "in" : "kitchen",
    "in_front_of_milk"      : milk_shelf,
    "lookat_milk"           : milk_shelf,
    "in_front_of_cereal"    : cereal_shelf,
    "lookat_cereal"         : cereal_shelf,
    "in_front_of_fruit"     : fruit_shelf,
    "lookat_fruit"          : fruit_shelf
}

dinner_table = "dinnertable"
