from __future__ import print_function

female_names = ["adel", "angel", "axel", "charlie", "jane", "jules", "morgan", "paris", "robin", "simone"]
male_names = ["adel", "angel", "axel", "charlie", "john", "jules", "morgan", "paris", "robin", "simone"]
names = set(female_names)
names.update(set(male_names))
names = sorted(list(names))

locations = [
    {"name": "bed", "room": "bedroom", "manipulation": False},
    {"name": "bedside_table", "room": "bedroom", "manipulation": True},
    {"name": "shelf", "room": "bedroom", "manipulation": True},
    {"name": "trashbin", "room": "kitchen", "manipulation": False},
    {"name": "dishwasher", "room": "kitchen", "manipulation": True},
    {"name": "potted_plant", "room": "kitchen", "manipulation": False},
    {"name": "kitchen_table", "room": "kitchen", "manipulation": True},
    {"name": "pantry", "room": "kitchen", "manipulation": True},
    {"name": "refrigerator", "room": "kitchen", "manipulation": True},
    {"name": "sink", "room": "kitchen", "manipulation": True},
    {"name": "cabinet", "room": "study", "manipulation": True},
    {"name": "coatrack", "room": "study", "manipulation": False},
    {"name": "desk", "room": "study", "manipulation": True},
    {"name": "armchair", "room": "study", "manipulation": False},
    {"name": "waste_basket", "room": "study", "manipulation": False},
    {"name": "tv_stand", "room": "living_room", "manipulation": True},
    {"name": "storage_rack", "room": "living_room", "manipulation": True},
    {"name": "lamp", "room": "living_room", "manipulation": False},
    {"name": "side_tables", "room": "living_room", "manipulation": True},
    {"name": "sofa", "room": "living_room", "manipulation": True},
    {"name": "bookshelf", "room": "living_room", "manipulation": True},
]

location_rooms = list(set([o["room"] for o in locations]))
location_names = list(set([o["name"] for o in locations]))
manipulation_locations = list(set([o["name"] for o in locations if o["manipulation"]]))

objects = [
    {"name": "sponge", "category": "cleaning_supplies"},
    {"name": "cleanser", "category": "cleaning_supplies"},
    {"name": "red_wine", "category": "drinks"},
    {"name": "juice_pack", "category": "drinks"},
    {"name": "cola", "category": "drinks"},
    {"name": "tropical_juice", "category": "drinks"},
    {"name": "milk", "category": "drinks"},
    {"name": "iced_tea", "category": "drinks"},
    {"name": "orange_juice", "category": "drinks"},
    {"name": "tuna", "category": "food"},
    {"name": "tomato_soup", "category": "food"},
    {"name": "spam", "category": "food"},
    {"name": "mustard", "category": "food"},
    {"name": "strawberry_jello", "category": "food"},
    {"name": "chocolate_jello", "category": "food"},
    {"name": "coffee_grounds", "category": "food"},
    {"name": "sugar", "category": "food"},
    {"name": "pear", "category": "fruits"},
    {"name": "plum", "category": "fruits"},
    {"name": "peach", "category": "fruits"},
    {"name": "lemon", "category": "fruits"},
    {"name": "orange", "category": "fruits"},
    {"name": "strawberry", "category": "fruits"},
    {"name": "banana", "category": "fruits"},
    {"name": "apple", "category": "fruits"},
    {"name": "tennis_ball", "category": "toys"},
    {"name": "soccer_ball", "category": "toys"},
    {"name": "rubiks_cube", "category": "toys"},
    {"name": "dice", "category": "toys"},
    {"name": "baseball", "category": "toys"},
    {"name": "pringles", "category": "snacks"},
    {"name": "cornflakes", "category": "snacks"},
    {"name": "cheezit", "category": "snacks"},
    {"name": "spoon", "category": "dishes"},
    {"name": "plate", "category": "dishes"},
    {"name": "cup", "category": "dishes"},
    {"name": "fork", "category": "dishes"},
    {"name": "bowl", "category": "dishes"},
    {"name": "knife", "category": "dishes"},
]

object_names = list(set([o["name"] for o in objects]))
object_categories = list(set([o["category"] for o in objects]))

category_locations = {
    "cleaning_supplies": {"shelf": "on_top_of"},
    "drinks": {"cabinet": "on_top_of"},
    "food": {"pantry": "on_top_of"},
    "fruits": {"desk": "on_top_of"},
    "toys": {"bookshelf": "on_top_of"},
    "snacks": {"side_tables": "on_top_of"},
    "dishes": {"kitchen_table": "on_top_of"},
}

inspect_areas = {
    "pantry": ["shelf2", "shelf3"],
}

inspect_positions = {
    'kitchen_shelf': {
        # 'shelf1_l': 'in_front_of_l',
        # 'shelf2_l': 'in_front_of_l',
        # 'shelf3_l': 'in_front_of_l',
        # 'shelf4_l': 'in_front_of_l',
        # 'shelf5_l': 'in_front_of_l',
        # 'shelf1_r': 'in_front_of_r',
        # 'shelf2_r': 'in_front_of_r',
        'shelf3_r': 'in_front_of_r',
        'shelf4_r': 'in_front_of_r',
        # 'shelf5_r': 'in_front_of_r',
        # 'on_top_of': 'in_front_of',
    }
}

drink_spec = "T['drink': O] -> OPTIONS[O]\n\n"
for drink in [obj["name"] for obj in objects if obj["category"] == "drinks"]:
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
    return entity_id in location_rooms


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


def is_manipulation_location(location):
    for loc in locations:
        if loc["name"] == location and loc["manipulation"] == True:
            return True
    return False


def get_locations(room=None, manipulation_location=None):
    return [loc["name"] for loc in locations
                if (room == None or loc["room"] == room) and \
                   (manipulation_location == None or manipulation_location == is_manipulation_location(loc["name"]))]


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
