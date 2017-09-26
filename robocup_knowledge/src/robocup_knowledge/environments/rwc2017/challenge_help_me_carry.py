commands = {
    'follow':               ["follow"],    # 'Robot, follow me'
    'follow_or_remember':   ["follow",     # 'Robot, follow me'
                             "remember",   # 'Remember, this location is the car'
                             "car",
                             "stop"],
    'carry':                ["carry",
                             "bag",
                             "bring"]}     # 'Carry this bag'

# add or get from ~/ros/kinetic/system/src/ed_object_models/models/rwc2017/model.yaml
waypoints = {
    'bedroom':              {'id':      "bedroom",
                             'radius':  1},
    'livingroom':           {'id':   "living_room",
                             'radius':  1},
    'kitchen':              {'id':   "kitchen",
                             'radius':  1},
    'balcony':              {'id':   "balcony",
                             'radius':  1},
    'corridor':             {'id':   "corridor",
                             'radius':  1},
    'entrance':             {'id':   "entrance",
                             'radius':  1},
    }

waypoint_car = {'id':      "car",
                'radius':  1}

default_item = "coke"
default_place = "kitchen_counter"
default_area = "on_top_of"

starting_point = "help_me_carry_start"
rotation = 0

time_out_seconds = 60.0
time_out_seconds_door = 120.0

