from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

starting_point = 'help_me_carry_starting_point'
rotation = 0

time_out_seconds = 60.0
time_out_seconds_door = 120.0

commands = {
    'no':  ['no', 'follow me'],# 'Robot, follow me'
    'yes': ['yes', 'remember location',  # 'Remember, this location is the car'
            'here is the car',
            'stop following',  # 'Stop following me'
            'stop following me']}

destinations = common.location_rooms + common.location_names

# add or get from ~/ros/kinetic/system/src/ed_object_models/models/robotics_testlabs/model.yaml
waypoint_car = {'id': 'car',
                'radius': 0.5}

default_item = 'coke'
default_place = 'dinner_table'
default_area = 'in_front_of'
default_target_radius = 0.2
backup_target_radius = 0.7

carrying_bag_pose = 'carrying_bag_pose'
drop_bag_pose = "drop_bag_pose"
driving_bag_pose = 'driving_bag_pose'
