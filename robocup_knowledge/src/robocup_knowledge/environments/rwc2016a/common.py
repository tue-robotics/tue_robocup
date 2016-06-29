from robocup_knowledge.environments.rwc2016_common.common import *

# Keep same reference
locations.extend([
    {"name": "bed", "room": "bedroom", "location_category": "", "manipulation": "no"},
    {"name": "desk", "room": "bedroom", "location_category": "snacks", "manipulation": "yes"},
    {"name": "bedside", "room": "bedroom", "location_category": "candies", "manipulation": "yes"},
    {"name": "bar", "room": "kitchen", "location_category": "", "manipulation": "yes"},
    {"name": "sink", "room": "kitchen", "location_category": "containers", "manipulation": "yes"},
    {"name": "cupboard", "room": "kitchen", "location_category": "drinks", "manipulation": "yes"},
    {"name": "sideshelf", "room": "kitchen", "location_category": "food", "manipulation": "yes"},
    {"name": "tv_stand", "room": "dining_room", "location_category": "", "manipulation": "yes"},
    {"name": "dining_table", "room": "dining_room", "location_category": "", "manipulation": "yes"},
    {"name": "bookcase", "room": "dining_room", "location_category": "", "manipulation": "yes"},
    {"name": "living_shelf", "room": "living_room", "location_category": "", "manipulation": "yes"},
    {"name": "living_table", "room": "living_room", "location_category": "", "manipulation": "yes"},
    {"name": "drawer", "room": "living_room", "location_category": "toiletries", "manipulation": "yes"},
    {"name": "cabinet", "room": "corridor", "location_category": "", "manipulation": "yes"}
])

category_locations.update({
    "candies":    ( "bedside",      "shelf3" ),
    "snacks":     ( "desk",         "on_top_of" ),
    "drinks":     ( "bookcase",     "on_top_of" ),
    "food":       ( "sideshelf",    "on_top_of" ),
    "toiletries": ( "living_shelf", "on_top_of" ),
    "containers": ( "sink",         "on_top_of" )
})

inspect_areas = {
    "bookcase" : ["shelf1", "shelf2", "shelf3", "shelf4", "shelf5"]
}

inspect_positions = {
}

rooms = list(set([o["room"] for o in locations]))
grab_locations = list(set([o["name"] for o in locations if o["manipulation"] == "yes"]))
put_locations = list(set([o["name"] for o in locations if o["manipulation"] != "no"]))

if __name__ == "__main__":
    print "\n-----------------------------------------------------------------------------"
    for obj in get_objects():
        cat = get_object_category(obj)
        print "object '{}'".format(obj)
        print "    category: '{}'".format(cat)
        (location, area_name) = get_object_category_location(cat)
        print "    found '{} {}'".format(area_name, location)

    print "\n-----------------------------------------------------------------------------"
    for loc in get_locations():
        print "location '{}', room: '{}'".format(loc, get_room(loc))

    print "\n-----------------------------------------------------------------------------"
    print "Pick locations:"
    for loc in get_locations(pick_location=True):
        print "    {}".format(loc)

    print "\n-----------------------------------------------------------------------------"
    print "Place locations:"
    for loc in get_locations(place_location=True):
        print "    {}".format(loc)


    print "\n-----------------------------------------------------------------------------"
    print "None-manipulation locations:"
    for loc in get_locations(pick_location=False, place_location=False):
        print "    {}".format(loc)
