
commands.follow             = ["follow"]    # 'Robot, follow me'

commands.follow_or_remember = ["follow",     # 'Robot, follow me'
                               "remember"]   # 'Remember, this location is the car'

commands.carry              = ["carry"]      # 'Carry this box'

commands.waypoint_for_room['kitchen'] = ('kitchen_table', 1) #TODO: update waypoint, target

waypoints.car.id = "car"
waypoints.car.radius = 1

starting_point = "initial_pose"
rotation = 0

time_out_seconds = 60.0
time_out_seconds_door = 120.0

