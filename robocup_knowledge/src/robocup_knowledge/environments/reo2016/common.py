# COMMON KNOWLEDGE FILE REO2016

female_names = ["Emma", "Olivia", "Sophia", "Isabella", "Ava", "Mia", "Emily", "Abigail", "Madison", "Charlotte"]
male_names = ["Noah", "Liam", "Mason", "Jacob", "William", "Ethan", "Michael", "Alexander", "James", "Daniel"]

names = female_names + male_names

objects = [
    {"name": "apple", "category": "food"},
    {"name": "avocado", "category": "food"},
    {"name": "bowl", "category": "container"},
    {"name": "chocolate_sprinkles", "category": "food"},
    {"name": "cloth", "category": "cleaning_stuff"},
    {"name": "dishwashing_soap", "category": "cleaning_stuff"},
    {"name": "kinder_coke", "category": "drink"},
    {"name": "lemon", "category": "food"},
    {"name": "licorice", "category": "candy"},
    {"name": "little_bananas", "category": "candy"},
    {"name": "macaroni", "category": "food"},
    {"name": "milk", "category": "drink"},
    {"name": "paprika", "category": "food"},
    {"name": "pineapple_cookies", "category": "candy"},
    {"name": "plate", "category": "container"},
    {"name": "rice", "category": "food"},
    {"name": "smoothie", "category": "drink"},
    {"name": "soap", "category": "cleaning_stuff"},
    {"name": "sponge", "category": "cleaning_stuff"},
    {"name": "storage_box", "category": "container"},
    {"name": "strawberry_cookies", "category": "candy"},
    {"name": "tea", "category": "drink"},
    {"name": "toilet_paper", "category": "cleaning_stuff"},
    {"name": "tuc", "category": "candy"},
    {"name": "wafer", "category": "candy"},
    {"name": "water", "category": "drink"}
]

object_names = list(set([o["name"] for o in objects]))
object_categories = list(set([o["category"] for o in objects]))

locations = [
    {"name": "closet",           "room": "bedroom",    "location_category": "shelf",     "manipulation": "yes"},
    {"name": "nightstand",       "room": "bedroom",    "location_category": "table",     "manipulation": "yes"},
    {"name": "bed",              "room": "bedroom",    "location_category": "seat",      "manipulation": "yes"},
    {"name": "fridge",           "room": "kitchen",    "location_category": "appliance", "manipulation": "no"},
    {"name": "kitchen_trashbin", "room": "kitchen",    "location_category": "bin",       "manipulation": "only_putting"},
    {"name": "kitchencounter",   "room": "kitchen",    "location_category": "table",     "manipulation": "yes"},
    {"name": "sink",             "room": "kitchen",    "location_category": "appliance", "manipulation": "only_putting"},
    {"name": "hallway_trashbin", "room": "kitchen",    "location_category": "bin",       "manipulation": "only_putting"},
    {"name": "sideboard",        "room": "livingroom", "location_category": "shelf",     "manipulation": "yes"},
    {"name": "dinnerchairs",     "room": "livingroom", "location_category": "seat",      "manipulation": "no"},
    {"name": "dinnertable",      "room": "livingroom", "location_category": "table",     "manipulation": "yes"},
    {"name": "bookcase",         "room": "livingroom", "location_category": "shelf",     "manipulation": "yes"},
    {"name": "couch",            "room": "livingroom", "location_category": "seat",      "manipulation": "yes"},
    {"name": "tv_stand",         "room": "livingroom", "location_category": "shelf",     "manipulation": "yes"}
]

rooms = list(set([o["room"] for o in locations]))
grab_locations = list(set([o["name"] for o in locations if o["manipulation"] == "yes"]))
put_locations = list(set([o["name"] for o in locations if o["manipulation"] != "no"]))

category_locations = {
    "cleaning_stuff": {"closet": "on_top_of"},
    "food": {"kitchencounter": "left_of_sink"},
    "drinks": {"kitchencounter": "right_of_sink"},
    "candy": {"sideboard": "on_top_of"},
    "containers": {"dinnertable": "on_top_of"}
}

def is_location(loc_name):
    for loc in locations:
        if loc["name"] == loc_name:
            return True
    return False

inspect_areas = {
    "closet" : ["on_top_of"],
    "bookcase" : ["shelf1", "shelf2", "shelf3", "shelf4"],
    "dinnertable" : ["on_top_of"],
    "kitchencounter" : ["left_of_sink", "right_of_sink"]
}

inspect_positions = {
    "kitchencounter" :
    {
        "left_of_sink"  : "in_front_of",
        "right_of_sink" : "in_front_of"
    }
}

def get_room(location):
    for loc in locations:
        if loc["name"] == location:
            return loc["room"]
    return None

def get_inspect_areas(location):
    if location in inspect_areas:
        return inspect_areas[location]
    else:
        return ["on_top_of"]

def get_inspect_position(location, area=""):
    if location in inspect_positions and area in inspect_positions[location]:
        return inspect_positions[location][area]
    else:
        return "in_front_of"
