# Tunable params
find_person = {
    'within_range' : 2.0,
    'under_z' : 0.3,
    'min_chull_area' : 0.06,
    'min_exist_prob' : 0.6
}

alarm_wait_time = 5
alarm_duration = 60

get_newspaper_timeout = 3
give_newspaper_timeout = 30
wakeup_light_color = [1, 1, 1]


# Knowledge
bed = 'bed'

bed_nav_goal = {
    'near' : bed,
    'in' : 'bedroom',
    'lookat' : 'bed'
}

default_milk = "fresh milk"

kitchen_nav_goal = {
    "in_front_of":"cabinet",
    "in" : "kitchen",
    "lookat" : "cabinet"
}

# Number of times it asks for the door to be opened and replans to the kitchen
check_door_attempts = 1


