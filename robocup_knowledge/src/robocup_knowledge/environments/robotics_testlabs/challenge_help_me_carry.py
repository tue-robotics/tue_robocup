from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

starting_point = 'help_me_carry_starting_point'
rotation = 0

time_out_seconds = 60.0
time_out_seconds_door = 120.0

commands = {
    'follow': ['follow',  # 'Robot, follow me'
               'follow me'],
    'remember': ['remember location',  # 'Remember, this location is the car'
                 'here is the car',
                 'stop following',  # 'Stop following me'
                 'stop following me']}

destinations = common.location_rooms+common.location_names

# add or get from ~/ros/kinetic/system/src/ed_object_models/models/robotics_testlabs/model.yaml
waypoint_car = {'id': 'car',
                'radius': 0.5}

default_item = 'coke'
default_place = 'cabinet'
default_area = 'on_top_of'
default_target_radius = 0.25

carrying_bag_pose = 'carrying_bag_pose'
drop_bag_pose = "drop_bag_pose"
driving_bag_pose = 'driving_bag_pose'
