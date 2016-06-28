# COMMON KNOWLEDGE FILE RWC2016A

female_names = ["Emma", "Taylor", "Sophia", "Isabella", "Ava", "Robin", "Emily", "Angel", "Madison", "Charlotte"]
male_names = ["Noah", "Liam", "Mason", "Jacob", "William", "Ethan", "Michael", "Alexander", "James", "Daniel"]
names = female_names + male_names

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

locations = []
category_locations = {}
inspect_areas = {}
inspect_positions = {}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

rooms = []
grab_locations = []
put_locations = []


def is_location(location):
    for loc in locations:
        if loc["name"] == location:
            return True
    return False


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


def is_pick_location(location):
    for loc in locations:
        if loc["name"] == location and loc["manipulation"] == "yes":
            return True
    return False


def is_place_location(location):
    for loc in locations:
        if loc["name"] == location and (loc["manipulation"] == "yes" or loc["manipulation"] == "only_putting"):
            return True
    return False


def get_locations(room=None, pick_location=None, place_location=None):
    return [loc["name"] for loc in locations if (room is None or loc["room"] == room) and
            (pick_location is None or is_pick_location(loc["name"])) and
            (place_location is None or is_place_location(loc["name"]))]

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

objects = []
object_names = []
object_categories = []


def get_objects(category=None):
    return [obj["name"] for obj in objects
                if category == None or category == obj["category"]]


def get_object_category(obj):
    for o in objects:
        if o["name"] == obj:
            return o["category"]
    return None


# Returns (location, area_name)
def get_object_category_location( obj_cat):
    location = category_locations[obj_cat].keys()[0]
    area_name = category_locations[obj_cat].values()[0]
    return location, area_name
