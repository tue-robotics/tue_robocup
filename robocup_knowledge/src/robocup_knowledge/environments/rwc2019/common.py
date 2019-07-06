# COMMON KNOWLEDGE FILE RWC2019

female_names = ["amelia", "angel", "ava", "charlie", "charlotte", "hunter", "max", "mia", "olivia", "parker", "sam"]
male_names = ["angel", "charlie", "hunter", "jack", "max", "noah", "oliver", "parker", "sam", "thomas", "william"]

names = female_names + male_names

# This dict holds all locations
locations = [
    {'name': 'bedroom_chest',   'room': 'bedroom',          'category': 'table',        'manipulation': 'yes'},
    {'name': 'bed',             'room': 'bedroom',          'category': 'utility',      'manipulation': 'no'},
    {'name': 'sidetable',      'room': 'bedroom',          'category': 'table',        'manipulation': 'yes'},
    {'name': 'shelf',           'room': 'bedroom',          'category': 'shelf',        'manipulation': 'yes'},

    {'name': 'trash_bin',       'room': 'kitchen',          'category': 'utility',      'manipulation': 'no'},
    {'name': 'kitchen_cabinet', 'room': 'kitchen',          'category': 'shelf',        'manipulation': 'yes'},
    {'name': 'kitchen_table',   'room': 'kitchen',          'category': 'table',        'manipulation': 'yes'},
    {'name': 'island',          'room': 'kitchen',          'category': 'table',        'manipulation': 'yes'},
    {'name': 'sink',            'room': 'kitchen',          'category': 'table',        'manipulation': 'yes'},
    {'name': 'dishwasher',      'room': 'kitchen',          'category': 'utility',      'manipulation': 'yes'},
    {'name': 'fridge',          'room': 'kitchen',          'category': 'utility',      'manipulation': 'no'},

    {'name': 'shoe_rack',       'room': 'office',           'category': 'table',        'manipulation': 'yes'},
    {'name': 'safe',            'room': 'office',           'category': 'table',        'manipulation': 'yes'},
    {'name': 'desk',            'room': 'office',           'category': 'table',        'manipulation': 'yes'},
    {'name': 'coat_hanger',     'room': 'office',           'category': 'utility',      'manipulation': 'no'},

    {'name': 'coffee_table',    'room': 'living_room',      'category': 'table',        'manipulation': 'yes'},
    {'name': 'couch',           'room': 'living_room',      'category': 'utility',      'manipulation': 'yes'},
    {'name': 'armchair',        'room': 'living_room',      'category': 'utility',      'manipulation': 'yes'},
    {'name': 'display_cabinet', 'room': 'living_room',      'category': 'table',        'manipulation': 'yes'},
    {'name': 'trash_bin1',      'room': 'living_room',      'category': 'utility',      'manipulation': 'yes'},
    {'name': 'sideboard',       'room': 'living_room',      'category': 'table',        'manipulation': 'yes'}
]

location_rooms = list(set([ o["room"] for o in locations ]))
location_categories = list(set([ o["category"] for o in locations ]))
location_names = list(set([ o["name"] for o in locations ]))
manipulation_locations = list(set([ o["name"] for o in locations if o["manipulation"] == "yes" ]))

objects = [
    {'category': 'candies',             'name': 'biscuit',         'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'candies',             'name': 'frosty_fruits',   'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'candies',             'name': 'snakes',          'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'cleaning_stuff',      'name': 'cloth',           'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'cleaning_stuff',      'name': 'dishwasher_tab',  'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'cleaning_stuff',      'name': 'sponge',          'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'cleaning_stuff',      'name': 'trash_bags',      'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'drink',               'name': 'beer',            'color': 'brown',       'volume': 270,      'weight': 222},
    {'category': 'drink',               'name': 'chocolate_milk',  'color': 'brownish',    'volume': 404,      'weight': 325},
    {'category': 'drink',               'name': 'coke',            'color': 'red',         'volume': 270,      'weight': 222},
    {'category': 'drink',               'name': 'juice',           'color': 'purplish',    'volume': 216,      'weight': 200},
    {'category': 'drink',               'name': 'lemonade',        'color': 'yellow',      'volume': 216,      'weight': 200},
    {'category': 'drink',               'name': 'tea_bag',         'color': 'bluegreen',   'volume': 270,      'weight': 222},
    {'category': 'drink',               'name': 'water',           'color': 'white',       'volume': 270,      'weight': 222},
    {'category': 'food',                'name': 'carrot',          'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'food',                'name': 'cereals',         'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'food',                'name': 'noodles',         'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'food',                'name': 'onion',           'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'food',                'name': 'vegemite',        'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'fruits',              'name': 'apple',           'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'fruits',              'name': 'kiwi',            'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'fruits',              'name': 'lemon',           'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'fruits',              'name': 'orange',          'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'fruits',              'name': 'pear',            'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'snacks',              'name': 'cheetos',         'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'snacks',              'name': 'doritos',         'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'snacks',              'name': 'shapes_chicken',  'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'snacks',              'name': 'shapes_pizza',    'color': 'blue',        'volume': 100,      'weight': 200},
    {'category': 'snacks',              'name': 'twisties',        'color': 'blue',        'volume': 100,      'weight': 200}
]

object_names = list(set([ o["name"] for o in objects ]))
object_categories = list(set([ o["category"] for o in objects ]))
object_color = list(set([ o["color"] for o in objects ]))
object_size = list(set([ o["volume"] for o in objects ]))
object_weight = list(set([ o["weight"] for o in objects ]))
# object_groups = list(set([ o["group"] for o in objects ]))
# object_known_objects = list(set([ o["name"] for o in objects ]))

category_locations = {
    "cleaning_stuff": {"sink": "on_top_of"},
    "containers": {"display_cabinet": "on_top_of"},
    "cutlery": {"kitchen_cabinet": "shelf2"},
    "drinks": {"sideboard": "on_top_of"},
    "candies": {"desk": "on_top_off"},
    "food": {"shelf": "on_top_of"},
    "fruits": {"kitchen_table": "on_top_off"},
    "snacks": {"coffee_table": "on_top_of"},
    "tableware": {"kitchen_cabinet": "shelf3"}
}

inspect_areas = {
    "shelf": ["shelf2", "shelf3", "shelf4", "shelf5"],
    "kitchen_cabinet": ["shelf1", "shelf2", "shelf3"]
}

inspect_positions = {
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

most_probable_location_in_room_map = {
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
