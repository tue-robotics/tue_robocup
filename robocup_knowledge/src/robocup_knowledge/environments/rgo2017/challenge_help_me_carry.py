default_item = "coke"
default_place = "couch_table"
default_area = "on_top_of"

commands = {
    'follow':               ["follow"],    # 'Robot, follow me'
    'follow_or_remember':   ["follow",     # 'Robot, follow me'
                             "remember",   # 'Remember, this location is the car'
                             "car",
                             "stop"],
    'carry':                ["carry",
                             "bag",
                             "bring"]}     # 'Carry this bag'

# add or get from ed_object_models/models/<world>/model.yaml
waypoints = {
    'kitchen':              {'id': 'explore1', 'radius':  1}  # TODO: update waypoint, target
}

waypoint_car = {'id': 'car', 'radius': 1}
