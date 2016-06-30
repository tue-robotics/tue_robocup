from robocup_knowledge.environments.rwc2016_common.common import *

# Keep same reference
locations.extend([
    { "room" : "bedroom",     "name" : "bed",          "location_category" : "",           "manipulation" : "no" },
    { "room" : "bedroom",     "name" : "desk",         "location_category" : "snacks",     "manipulation" : "yes" },
    { "room" : "bedroom",     "name" : "bedside",      "location_category" : "candies",    "manipulation" : "yes" },
    { "room" : "kitchen",     "name" : "bar",          "location_category" : "",           "manipulation" : "yes" },
    { "room" : "kitchen",     "name" : "sink",         "location_category" : "containers", "manipulation" : "yes" },
    { "room" : "kitchen",     "name" : "cupboard",     "location_category" : "drinks",     "manipulation" : "yes" },
    { "room" : "kitchen",     "name" : "sideshelf",    "location_category" : "food",       "manipulation" : "yes" },
    { "room" : "dining_room", "name" : "tv_stand",     "location_category" : "",           "manipulation" : "yes" },
    { "room" : "dining_room", "name" : "dining_table", "location_category" : "",           "manipulation" : "yes" },
    { "room" : "dining_room", "name" : "bookcase",     "location_category" : "",           "manipulation" : "yes" },
    { "room" : "living_room", "name" : "living_shelf", "location_category" : "",           "manipulation" : "yes" },
    { "room" : "living_room", "name" : "living_table", "location_category" : "",           "manipulation" : "yes" },
    { "room" : "living_room", "name" : "drawer",       "location_category" : "toiletries", "manipulation" : "yes" },
    { "room" : "corridor",    "name" : "cabinet",      "location_category" : "",           "manipulation" : "yes" }
])

category_locations.update({
    "candies":    ( "bedside",      "on_top_of" ),
    "snacks":     ( "desk",         "on_top_of" ),
    "drinks":     ( "cupboard",     "on_top_of" ),
    "food":       ( "sideshelf",    "on_top_of" ),
    "toiletries": ( "drawer",       "on_top_of" ),
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
    test_knowledge()