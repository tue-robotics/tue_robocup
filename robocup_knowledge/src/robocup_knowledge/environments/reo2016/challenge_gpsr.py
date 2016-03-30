from itertools import groupby
from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

rooms = common.rooms

object_aliases = {"dinner table" : "dinnertable"}

# mapping from furniture to small objects that are on top of them (and can be grabbed)
furniture_to_objects = {
    "cabinet": [
        "beer",
        "bifrutas",
        "coffee_pads",
        "coke",
        "deodorant"
    ],

    "dinnertable": [
        "fanta",
        "ice_tea",
        "mentos",
        "sprite",
        "tea",
        "teddy_bear",
        "water",
        "xylit24_spearmint",
        "xylit24_white"],

    "bed": []
}

_grab_locations = [o for o in common.locations if o["manipulation"] == "yes"]
room_to_grab_locations = groupby(_grab_locations, lambda l: l['room'])
room_to_grab_locations = {k: [l['name'] for l in g] for (k, g) in room_to_grab_locations}
