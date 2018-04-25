## From robotics_testlabs, not sure if correct
from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

starting_point = 'help_me_carry_starting_point'
rotation = 0

time_out_seconds = 60.0
time_out_seconds_door = 120.0

destinations = common.location_rooms + common.location_names

default_target_radius = 0.2
backup_target_radius = 0.7

default_item = "coke"
default_place = "couch_table"
default_area = "on_top_of"
starting_point = "initial_pose" # copied from RIPS so perhaps not correct

commands = {                # Copied from robotics_testlabs, I think the commands can stay the same.
    'no':  ['no', 'follow me'],# 'Robot, follow me'
    'yes': ['yes', 'remember location',  # 'Remember, this location is the car'
            'here is the car',
            'stop following',  # 'Stop following me'
            'stop following me']}

# add or get from ed_object_models/models/<world>/model.yaml
waypoints = {
    'kitchen':              {'id': 'explore1', 'radius':  1}  # TODO: update waypoint, target
}

waypoint_car = {'id': 'car', 'radius': 1}
