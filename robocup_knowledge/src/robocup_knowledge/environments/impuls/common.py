# COMMON KNOWLEDGE FILE ROBOTICS TESTLABS
from __future__ import print_function

female_names = ["anna", "beth", "carmen", "jennifer", "jessica", "kimberly", "kristina", "laura", "mary", "sarah"]
male_names = ["alfred", "charles", "daniel", "james", "john", "luis", "paul", "richard", "robert", "steve"]

names = female_names + male_names

# This dict holds all locations
locations = [
    {'name': 'dinner_table',  'room': 'livingroom', 'category': 'table',   'manipulation': 'yes'},
    {'name': 'bookcase',      'room': 'livingroom', 'category': 'shelf',   'manipulation': 'yes'},
    {'name': 'cabinet',       'room': 'livingroom', 'category': 'shelf',   'manipulation': 'yes'},
    {'name': 'closet',        'room': 'livingroom', 'category': 'shelf',   'manipulation': 'yes'},

    {'name': 'kitchen_cabinet', 'room': 'kitchen',    'category': 'utility', 'manipulation': 'no'},
    {'name': 'fridge',        'room': 'kitchen',    'category': 'utility', 'manipulation': 'no'},
    {'name': 'salon_table',   'room': 'kitchen',    'category': 'table', 'manipulation': 'yes'},
    {'name': 'couch_long',    'room': 'kitchen',    'category': 'utility', 'manipulation': 'no'},
    {'name': 'couch_short',   'room': 'kitchen',    'category': 'utility', 'manipulation': 'no'},

    {'name': 'workbench1',     'room': 'workshop',   'category': 'table',   'manipulation': 'yes'},
    {'name': 'workbench2',     'room': 'workshop',   'category': 'table',   'manipulation': 'yes'},
]

location_rooms = list(set([ o["room"] for o in locations ]))
location_categories = list(set([ o["category"] for o in locations ]))
location_names = list(set([ o["name"] for o in locations ]))
manipulation_locations = list(set([ o["name"] for o in locations if o["manipulation"] == "yes" ]))

rooms = location_rooms + ["workshop"]

objects = [
    {'category': 'food',                'name': 'apple',            'color': 'green',           'volume': 301,      'weight': 100},  # fruit
    {'category': 'drink',               'name': 'coke',             'color': 'red',             'volume': 376,      'weight': 335},
    {'category': 'drink',               'name': 'fanta',            'color': 'orange',          'volume': 376,      'weight': 335},
    {'category': 'drink',               'name': 'ice_tea',          'color': 'yellow',          'volume': 376,      'weight': 335}
]

object_names = list(set([ o["name"] for o in objects ]))
object_categories = list(set([ o["category"] for o in objects ]))
object_color = list(set([ o["color"] for o in objects ]))
object_size = list(set([ o["volume"] for o in objects ]))
object_weight = list(set([ o["weight"] for o in objects ]))
# object_groups = list(set([ o["group"] for o in objects ]))
# object_known_objects = list(set([ o["name"] for o in objects ]))

category_locations = {
    "food": {"closet": "on_top_of"},
    "snack": {"closet": "on_top_of"},
    "container": {"cabinet": "on_top_of"},
    "drink": {"dinner_table": "on_top_of"},
    "cleaning_stuff": {"cabinet": "on_top_of"},
    "cutlery": {"cabinet": "on_top_of"}
}

inspect_areas = {
    "closet": ["shelf1", "shelf2", "shelf3", "shelf4", "shelf5"],
    "bookcase": ["shelf1", "shelf2", "shelf3", "shelf4"]
}

inspect_positions = {
}

default_target_radius = 0.2

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

most_probable_location_in_room_map = {
    'livingroom': 'dinner_table',
    'workshop': 'workbench1',
    'kitchen': 'salon_table'
}


def get_location_from_room(room_id):
    if room_id in most_probable_location_in_room_map:
        return most_probable_location_in_room_map[room_id]
    return None


def object_names_of_category(category):
    return [obj['name'] for obj in objects if obj['category'] == category]


drink_names = object_names_of_category('drink')
drink_spec = "T['drink': O] -> OPTIONS[O]\n\n"
for dn in drink_names:
    drink_spec += "OPTIONS['{drink}'] -> {drink}\n".format(drink=dn)

''' colors from printing on screen '''
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


'''
General function for printing shortcuts
name: name of the program that instantiates make_prints
sentence: sentence to be displayed

Ex: "[<EXECUTIVE NAME>] <SENTENCE TO BE DISPLAYED>"
'''


def make_prints(name):

    prefix = bcolors.HEADER + name + bcolors.ENDC

    def printOk(sentence):
        print(prefix + bcolors.OKBLUE + sentence + bcolors.ENDC)

    def printError(sentence):
        print(prefix + bcolors.FAIL + sentence + bcolors.ENDC)

    def printWarning(sentence):
        print(prefix + bcolors.WARNING + sentence + bcolors.ENDC)

    return printOk, printError, printWarning

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


if __name__ == "__main__":
    print("\n-----------------------------------------------------------------------------")
    for obj in get_objects():
        cat = get_object_category(obj)
        (location, area_name) = get_object_category_location(cat)
        print("object '{}'".format(obj))
        print("    category: '{}'".format(cat))
        print("    found '{} {}'".format(area_name, location))

    print("\n-----------------------------------------------------------------------------")
    for loc in get_locations():
        print("location '{}', room: '{}'".format(loc, get_room(loc)))

    print("\n-----------------------------------------------------------------------------")
    print("Pick locations:")
    for loc in get_locations(pick_location=True):
        print("    {}".format(loc))

    print("\n-----------------------------------------------------------------------------")
    print("Place locations:")
    for loc in get_locations(place_location=True):
        print("    {}".format(loc))

    print("\n-----------------------------------------------------------------------------")
    print("None-manipulation locations:")
    for loc in get_locations(pick_location=False, place_location=False):
        print("    {}".format(loc))
