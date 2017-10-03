commands = {
    'follow':               ["follow"],    # 'Robot, follow me'
    'follow_or_remember':   ["follow",     # 'Robot, follow me'
                             "remember",   # 'Remember, this location is the car'
                             "car",
                             "stop"],
    'carry':                ["carry",
                             "bag",
                             "bring"]}     # 'Carry this bag'

# add or get from ~/ros/kinetic/system/src/ed_object_models/models/robotics_testlabs/model.yaml
waypoints = {
    'bedroom':              {'id':      "explore6",
                             'radius':  1},
    'livingroom':              {'id':   "operator_pose",
                             'radius':  1},
    }

waypoint_car = {'id':      "car",
                'radius':  1}

default_item = "coke"
default_place = "cabinet"
default_area = "on_top_of"

starting_point = "help_me_carry_starting_point"
rotation = 0

time_out_seconds = 60.0
time_out_seconds_door = 120.0

