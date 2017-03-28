commands = {
    'follow':               ["follow"],    # 'Robot, follow me'
    'follow_or_remember':   ["follow",     # 'Robot, follow me'
                             "remember"],   # 'Remember, this location is the car'
    'carry':                ["carry"]}      # 'Carry this box'

waypoints = {
    'kitchen':              {'id':      "kitchen_table",
                             'radius':  1}, #TODO: update waypoint, target
    'car':                  {'id':      "car",
                             'radius':  1}}

starting_point = "initial_pose"
rotation = 0

time_out_seconds = 60.0
time_out_seconds_door = 120.0

