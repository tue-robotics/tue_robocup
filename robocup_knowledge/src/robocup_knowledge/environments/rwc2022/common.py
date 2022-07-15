from __future__ import print_function

female_names = ["amelia", "angel", "ava", "charlie", "charlotte", "hunter", "max", "mia", "olivia", "parker", "sam"]
male_names = ["angel", "charlie", "hunter", "jack", "max", "noah", "oliver", "parker", "sam", "thomas", "william"]
names = female_names + male_names

locations = [
    {"name": "house_plant", "room": "living_room", "manipulation": False},
    {"name": "coat_rack", "room": "living_room", "manipulation": False},
    {"name": "sofa", "room": "living_room", "manipulation": False},
    {"name": "couch_table", "room": "living_room", "manipulation": True},
    {"name": "tv", "room": "living_room", "manipulation": False},
    {"name": "side_table", "room": "living_room", "manipulation": True},
    {"name": "book_shelf", "room": "living_room", "manipulation": True},
    {"name": "kitchen_shelf", "room": "kitchen", "manipulation": True},
    {"name": "pantry", "room": "kitchen", "manipulation": True},
    {"name": "dinner_table", "room": "kitchen", "manipulation": True},
    {"name": "kitchen_bin", "room": "kitchen", "manipulation": True},
    {"name": "fridge", "room": "kitchen", "manipulation": True},
    {"name": "washing_machine", "room": "kitchen", "manipulation": True},
    {"name": "sink", "room": "kitchen", "manipulation": False},
    {"name": "small_shelf", "room": "bedroom", "manipulation": True},
    {"name": "cupboard", "room": "bedroom", "manipulation": True},
    {"name": "big_shelf", "room": "bedroom", "manipulation": True},
    {"name": "bed", "room": "bedroom", "manipulation": False},
    {"name": "desk", "room": "office", "manipulation": True},
    {"name": "show_rack", "room": "office", "manipulation": False},
    {"name": "bin", "room": "office", "manipulation": False},
    {"name": "office_shelf", "room": "office", "manipulation": True},
]

location_rooms = list(set([o["room"] for o in locations]))
rooms = location_rooms
location_names = list(set([o["name"] for o in locations]))
manipulation_locations = list(set([o["name"] for o in locations if o["manipulation"]]))

objects = [
    {'name': 'water', 'category': 'drink'},
    {'name': 'milk', 'category': 'drink'},
    {'name': 'coke', 'category': 'drink'},
    {'name': 'tonic', 'category': 'drink'},
    {'name': 'bubble_tea', 'category': 'drink'},
    {'name': 'ice_tea', 'category': 'drink'},
    {'name': 'cloth', 'category': 'cleaning_supply'},
    {'name': 'sponge', 'category': 'cleaning_supply'},
    {'name': 'cleaner', 'category': 'cleaning_supply'},
    {'name': 'corn_flakes', 'category': 'pantry_item'},
    {'name': 'tuna_can', 'category': 'pantry_item'},
    {'name': 'coffee_jar', 'category': 'pantry_item'},
    {'name': 'sugar', 'category': 'pantry_item'},
    {'name': 'mustard', 'category': 'pantry_item'},
    {'name': 'apple', 'category': 'fruit'},
    {'name': 'peach', 'category': 'fruit'},
    {'name': 'orange', 'category': 'fruit'},
    {'name': 'banana', 'category': 'fruit'},
    {'name': 'strawberry', 'category': 'fruit'},
    {'name': 'pockys', 'category': 'snack'},
    {'name': 'pringles', 'category': 'snack'},
    {'name': 'spoon', 'category': 'cutlery'},
    {'name': 'fork', 'category': 'cutlery'},
    {'name': 'plate', 'category': 'cutlery'},
    {'name': 'bowl', 'category': 'cutlery'},
    {'name': 'mug', 'category': 'cutlery'},
    {'name': 'knife', 'category': 'cutlery'},
]

object_names = list(set([o["name"] for o in objects]))
object_categories = list(set([o["category"] for o in objects]))

category_locations = {
    "fruit": {"couch_table": "on_top_of"},
    "drink": {"side_table": "on_top_of"},
    "pantry_item": {"pantry": "on_top_of"},
    "cutlery": {"dinner_table": "on_top_of"},
    "cleaning_supply": {"small_shelf": "on_top_of"},
    "snack": {"desk": "on_top_of"}
}

inspect_areas = {
    "kitchen_shelf": ["shelf2_r", "shelf3_r", "shelf4_r"],
}

inspect_positions = {
    'kitchen_shelf': {
        # 'shelf1_l': 'in_front_of_l',
        # 'shelf2_l': 'in_front_of_l',
        # 'shelf3_l': 'in_front_of_l',
        # 'shelf4_l': 'in_front_of_l',
        # 'shelf5_l': 'in_front_of_l',
        'shelf1_r': 'in_front_of_r',
        'shelf2_r': 'in_front_of_r',
        'shelf3_r': 'in_front_of_r',
        'shelf4_r': 'in_front_of_r',
        'shelf5_r': 'in_front_of_r',
        'on_top_of': 'in_front_of',
    }
}

drink_spec = "T['drink': O] -> OPTIONS[O]\n\n"
for drink in [obj["name"] for obj in objects if obj["category"] == "drink"]:
    drink_spec += "OPTIONS['{drink}'] -> {drink}\n".format(drink=drink)


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


def is_room(entity_id):
    return (entity_id in rooms)


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
    return [loc["name"] for loc in locations
                if (room == None or loc["room"] == room) and \
                   (pick_location == None or pick_location == is_pick_location(loc["name"])) and \
                   (place_location == None or place_location == is_place_location(loc["name"]))]


def is_known_object(obj):
    for o in objects:
        if o["name"] == obj:
            return True
    return False


def get_objects(category=None):
    return [obj["name"] for obj in objects
                if category == None or category == obj["category"]]


def get_object_category(obj):
    for o in objects:
        if o["name"] == obj:
            return o["category"]
    return None

def get_object_color(obj):
    for o in objects:
        if o["name"] == obj:
            return o["color"]
    return None

def get_object_size(obj):
    for o in objects:
        if o["name"] == obj:
            return o["volume"]
    return None

def get_object_weight(obj):
    for o in objects:
        if o["name"] == obj:
            return o["weight"]
    return None

# Returns (location, area_name)
def get_object_category_location(obj_cat):
    location, area_name = next(iter(category_locations[obj_cat].items()))
    return location, area_name
