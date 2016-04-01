from itertools import groupby
from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

# initial pose
starting_point = "initial_pose_door_A"
exit_waypoint = "exit_door_B1"

rooms = common.rooms

#object_aliases = {"dinner table" : "dinnertable"}

#_grab_locations = [o for o in common.locations if o["manipulation"] == "yes"]
# room_to_grab_locations = groupby(_grab_locations, lambda l: l['room'])
# room_to_grab_locations = {k: [l['name'] for l in g] for (k, g) in room_to_grab_locations}
