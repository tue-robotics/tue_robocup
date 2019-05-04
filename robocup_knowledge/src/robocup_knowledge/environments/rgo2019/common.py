# COMMON KNOWLEDGE FILE RGO2019

# Names

female_names = ["sophia","isabella","emma","olivia","ava","emily","abigail","madison","mia","chloe"]
male_names = ["james","john","robert","michael","william","david","richard","charles","joseph","thomas"]

names = female_names + male_names

# This dict holds all locations
locations = [
    {'name': 'bed',             'room': 'bedroom',          'category': 'utility',      'manipulation': 'no'},
    {'name': 'desk',            'room': 'bedroom',          'category': 'table',        'manipulation': 'yes'},
    {'name': 'side_table',      'room': 'bedroom',          'category': 'table',        'manipulation': 'yes'},

    {'name': 'kitchen_table',   'room': 'kitchen',          'category': 'table',        'manipulation': 'yes'},
    {'name': 'kitchen_cabinet', 'room': 'kitchen',          'category': 'shelf',        'manipulation': 'yes'},
    {'name': 'dishwasher',      'room': 'kitchen',          'category': 'utility',      'manipulation': 'yes'},
    {'name': 'cabinet',         'room': 'kitchen',          'category': 'shelf',        'manipulation': 'yes'},
    {'name': 'white_drawer',    'room': 'kitchen',          'category': 'shelf',        'manipulation': 'yes'},
    {'name': 'trash_can',       'room': 'kitchen',          'category': 'utility',      'manipulation': 'yes'},

    {'name': 'tv_table',        'room': 'living_room',      'category': 'table',        'manipulation': 'yes'},
    {'name': 'tv',              'room': 'living_room',      'category': 'utility',      'manipulation': 'no'},
    {'name': 'bookcase',        'room': 'living_room',      'category': 'shelf',        'manipulation': 'yes'},
    {'name': 'couch',           'room': 'living_room',      'category': 'utility',      'manipulation': 'no'},
    {'name': 'left_armchair',   'room': 'living_room',      'category': 'utility',      'manipulation': 'no'},
    {'name': 'right_armchair',  'room': 'living_room',      'category': 'utility',      'manipulation': 'no'},
    {'name': 'coffee_table',    'room': 'living_room',      'category': 'table',        'manipulation': 'yes'},
    {'name': 'sideboard',       'room': 'living_room',      'category': 'table',        'manipulation': 'yes'},
    {'name': 'high_table',      'room': 'living_room',      'category': 'table',        'manipulation': 'yes'},
    {'name': 'trash_bin',       'room': 'living_room',      'category': 'utility',      'manipulation': 'yes'},
    {'name': 'coathanger',      'room': 'living_room',      'category': 'utility',      'manipulation': 'no'},


    {'name': 'bar_table',       'room': 'bar',              'category': 'table',        'manipulation': 'yes'},
    {'name': 'cupboard',        'room': 'bar',              'category': 'shelf',        'manipulation': 'yes'},
    {'name': 'sofa',            'room': 'bar',              'category': 'utility',      'manipulation': 'no'}
]

location_rooms = list(set([ o["room"] for o in locations ]))
location_categories = list(set([ o["category"] for o in locations ]))
location_names = list(set([ o["name"] for o in locations ]))
manipulation_locations = list(set([ o["name"] for o in locations if o["manipulation"] == "yes" ]))

rooms = location_rooms + ['hallway']

objects = [
    {'category': 'other',               'name': 'trashbag',                         'color': 'black',       'volume': 100,      'weight': 22},

    {'category': 'kitchen_stuff',       'name': 'bowl',                             'color': 'green',       'volume': 458,      'weight': 65},
    {'category': 'kitchen_stuff',       'name': 'cup',                              'color': 'yellow',      'volume': 300,      'weight': 50},
    {'category': 'kitchen_stuff',       'name': 'fork',                             'color': 'orange',      'volume': 22,       'weight': 10},
    {'category': 'kitchen_stuff',       'name': 'knife',                            'color': 'green',       'volume': 22,       'weight': 10},
    {'category': 'kitchen_stuff',       'name': 'plate',                            'color': 'blue',        'volume': 672,      'weight': 70},
    {'category': 'kitchen_stuff',       'name': 'spoon',                            'color': 'blue',        'volume': 23,       'weight': 10},

    {'category': 'drink',               'name': 'apple_juice',                      'color': 'brown',       'volume': 216,      'weight': 200},
    {'category': 'drink',               'name': 'big_coke',                         'color': 'brownish',    'volume': 270,      'weight': 222},
    {'category': 'drink',               'name': 'big_lemon_juice',                  'color': 'transparent', 'volume': 270,      'weight': 222},    
    {'category': 'drink',               'name': 'big_water',                        'color': 'transparent', 'volume': 270,      'weight': 222},
    {'category': 'drink',               'name': 'iso_drink',                        'color': 'blue',        'volume': 404,      'weight': 325},
    {'category': 'drink',               'name': 'milk',                             'color': 'blue',        'volume': 216,      'weight': 200},
    {'category': 'drink',               'name': 'orange_juice',                     'color': 'orange',      'volume': 216,      'weight': 200},
    {'category': 'drink',               'name': 'red_spritzer',                     'color': 'red',         'volume': 270,      'weight': 222},
    {'category': 'drink',               'name': 'sparkling_water',                  'color': 'transparent', 'volume': 270,      'weight': 222},

    {'category': 'fruit',               'name': 'lemon',                            'color': 'yellow',      'volume': 360,      'weight': 85},
    {'category': 'fruit',               'name': 'orange',                           'color': 'orange',      'volume': 380,      'weight': 140},

    {'category': 'care',                'name': 'shower_gel',                       'color': 'blue',        'volume': 336,      'weight': 21},
    {'category': 'care',                'name': 'soap',                             'color': 'ivory',       'volume': 297,      'weight': 85},
    {'category': 'care',                'name': 'toothpaste',                       'color': 'whitegreen',  'volume': 223,      'weight': 113},

    {'category': 'food',                'name': 'bouillon',                         'color': 'orange',      'volume': 336,      'weight': 21},
    {'category': 'food',                'name': 'corn',                             'color': 'yellow',      'volume': 223,      'weight': 113},
    {'category': 'food',                'name': 'noodles',                          'color': 'yellowish',   'volume': 297,      'weight': 85},
    {'category': 'food',                'name': 'pepper',                           'color': 'blue',        'volume': 336,      'weight': 21},
    {'category': 'food',                'name': 'salt',                             'color': 'whiteorange', 'volume': 297,      'weight': 85},
    {'category': 'food',                'name': 'sauerkraut',                       'color': 'whitegreen',  'volume': 223,      'weight': 113},
    {'category': 'food',                'name': 'seasoning_mix',                    'color': 'orange',      'volume': 297,      'weight': 85},
    {'category': 'food',                'name': 'tomatoes',                         'color': 'blue',        'volume': 223,      'weight': 113},

    {'category': 'container',           'name': 'basket',                           'color': 'white',       'volume': 3487,     'weight': 43},
    {'category': 'container',           'name': 'tray',                             'color': 'gray',        'volume': 4508,     'weight': 120},

    {'category': 'cleaning_stuff',      'name': 'cloth',                            'color': 'yellow',      'volume': 315,      'weight': 37},
    {'category': 'cleaning_stuff',      'name': 'dishwasher_tab',                   'color': 'yellowish',   'volume': 100,      'weight': 22},

    {'category': 'snack',               'name': 'cereal_bar_chocolate',             'color': 'blue',        'volume': 243,      'weight': 28},
    {'category': 'snack',               'name': 'cereal_bar_chocolate_banana',      'color': 'yellow',      'volume': 4080,     'weight': 200},
    {'category': 'snack',               'name': 'cracker',                          'color': 'redwhite',    'volume': 1125,     'weight': 156},
    {'category': 'kitchen_stuff',       'name': 'fruit_bar_apple',                  'color': 'green',       'volume': 458,      'weight': 65},
    {'category': 'kitchen_stuff',       'name': 'fruit_bar_forest_fruit',           'color': 'blue',        'volume': 300,      'weight': 50},
    {'category': 'kitchen_stuff',       'name': 'get_it',                           'color': 'orange',      'volume': 22,       'weight': 10},
    {'category': 'kitchen_stuff',       'name': 'nut_fruit_mix',                    'color': 'redwhite',    'volume': 22,       'weight': 10},
    {'category': 'kitchen_stuff',       'name': 'peanut_bits',                      'color': 'red',         'volume': 672,      'weight': 70}
]

object_names = list(set([ o["name"] for o in objects ]))
object_categories = list(set([ o["category"] for o in objects ]))
object_color = list(set([ o["color"] for o in objects ]))
object_size = list(set([ o["volume"] for o in objects ]))
object_weight = list(set([ o["weight"] for o in objects ]))
# object_groups = list(set([ o["group"] for o in objects ]))
# object_known_objects = list(set([ o["name"] for o in objects ]))

category_locations = {
    "cleaning_stuff": {"dishwasher": "on_top_of"},
    "snack": {"coffee_table": "on_top_of"},
    "drink": {"kitchen_table": "on_top_of"},
    "food": {"bookcase": "shelf2"},
    "care": {"side_table": "on_top_of"},
    "container": {"end_table": "on_top_of"},
    "fruit": {"bar_table": "on_top_of"},
    "kitchen_stuff": {"kitchen_cabinet": "on_top_of"},
}

inspect_areas = {
    "bookcase": ["shelf1", "shelf2"]
}

inspect_positions = {
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

most_probable_location_in_room_map = {
    'bedroom': 'side_table',        # educated guess
    'kitchen': 'kitchen_cabinet',     # educated guess
    'bar': 'bar_table',             # educated guess
    'living_room': 'bookcase'       # educated guess
}

def get_location_from_room(room_id):
    if room_id in most_probable_location_in_room_map:
        return most_probable_location_in_room_map[room_id]
    return None


drink_names = [obj['name'] for obj in objects if obj['category'] == 'drink']
drink_spec = "T['drink': O] -> OPTIONS[O]\n\n"
for dn in drink_names:
    drink_spec += "OPTIONS['{drink}'] -> {drink}\n".format(drink=dn)
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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
        print prefix + bcolors.OKBLUE + sentence + bcolors.ENDC

    def printError(sentence):
        print prefix + bcolors.FAIL + sentence + bcolors.ENDC

    def printWarning(sentence):
        print prefix + bcolors.WARNING + sentence + bcolors.ENDC

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
    location = category_locations[obj_cat].keys()[0]
    area_name = category_locations[obj_cat].values()[0]
    return (location, area_name)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

if __name__ == "__main__":
    print "\n-----------------------------------------------------------------------------"
    for obj in get_objects():
        cat = get_object_category(obj)
        (location, area_name) = get_object_category_location(cat)
        print "object '{}'".format(obj)
        print "    category: '{}'".format(cat)
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
