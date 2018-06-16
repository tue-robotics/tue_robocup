# COMMON KNOWLEDGE FILE ROBOTICS TESTLABS

female_names = ["anna", "beth", "carmen", "jennifer", "jessica", "kimberly", "kristina", "laura", "mary", "sarah"]
male_names = ["alfred", "charles", "daniel", "james", "john", "luis", "paul", "richard", "robert", "steve"]

names = female_names + male_names

# This dict holds all locations
locations = [
    {'name': 'couch_table',   'room': 'livingroom', 'category': 'table',   'manipulation': 'yes'},
    {'name': 'dinner_table',  'room': 'livingroom', 'category': 'table',   'manipulation': 'yes'},
    {'name': 'bar',           'room': 'livingroom', 'category': 'table',   'manipulation': 'yes'},
    {'name': 'bookcase',      'room': 'livingroom', 'category': 'shelf',   'manipulation': 'yes'},

    {'name': 'cabinet',       'room': 'kitchen',    'category': 'shelf',   'manipulation': 'yes'},
    {'name': 'trashbin',      'room': 'kitchen',    'category': 'utility', 'manipulation': 'no'},
    {'name': 'plant',         'room': 'kitchen',    'category': 'plant',   'manipulation': 'no'},

    {'name': 'bed',           'room': 'bedroom',    'category': 'seat',    'manipulation': 'yes'},
    {'name': 'nightstand',    'room': 'bedroom',    'category': 'table',   'manipulation': 'yes'},

    {'name': 'flight_case',   'room': 'workshop',   'category': 'table',   'manipulation': 'no'},
    {'name': 'battery_table', 'room': 'workshop',   'category': 'table',   'manipulation': 'no'},
    {'name': 'workbench',     'room': 'workshop',   'category': 'table',   'manipulation': 'yes'},

    {'name': 'hallway_table', 'room': 'hallway',    'category': 'table',   'manipulation': 'yes'}
]

location_rooms = list(set([ o["room"] for o in locations ]))
location_categories = list(set([ o["category"] for o in locations ]))
location_names = list(set([ o["name"] for o in locations ]))
manipulation_locations = list(set([ o["name"] for o in locations if o["manipulation"] == "yes" ]))

rooms = location_rooms + ["workshop"]

objects = [
    {'category': 'food',                'name': 'apple',            'color': 'green',           'volume': 301,      'weight': 100},  # fruit
    {'category': 'food',                'name': 'banana',           'color': 'yellow',          'volume': 320,      'weight': 100},  # fruit
    {'category': 'food',                'name': 'mango',            'color': 'red',             'volume': 254,      'weight': 100},  # fruit
    {'category': 'food',                'name': 'orange',           'color': 'orange',          'volume': 352,      'weight': 100},  # fruit
    {'category': 'food',                'name': 'cereal',           'color': 'green',           'volume': 1485,     'weight': 100},
    {'category': 'food',                'name': 'corn',             'color': 'yellow',          'volume': 240,      'weight': 100},
    {'category': 'food',                'name': 'crackers',         'color': 'blueish',         'volume': 300,      'weight': 100},
    {'category': 'food',                'name': 'peas',             'color': 'green',           'volume': 239,      'weight': 100},
    {'category': 'food',                'name': 'pepper',           'color': 'black',           'volume': 110,      'weight': 100},
    {'category': 'food',                'name': 'salt',             'color': 'white',           'volume': 110,      'weight': 100},
    {'category': 'food',                'name': 'pasta',            'color': 'yellow',          'volume': 1570,     'weight': 100},
    {'category': 'container',           'name': 'basket',           'color': 'green',           'volume': 3487,     'weight': 100},
    {'category': 'container',           'name': 'bowl',             'color': 'blue',            'volume': 617,      'weight': 100},
    {'category': 'container',           'name': 'cup',              'color': 'orangish',        'volume': 427,      'weight': 100},
    {'category': 'container',           'name': 'plate',            'color': 'grey',            'volume': 147,      'weight': 100},
    {'category': 'drink',               'name': 'beer',             'color': 'brown',           'volume': 388,      'weight': 100},
    {'category': 'drink',               'name': 'bifrutas',         'color': 'blue',            'volume': 378,      'weight': 100},
    {'category': 'drink',               'name': 'coke',             'color': 'red',             'volume': 376,      'weight': 335},
    {'category': 'drink',               'name': 'fanta',            'color': 'orange',          'volume': 376,      'weight': 335},
    {'category': 'drink',               'name': 'ice_tea',          'color': 'yellow',          'volume': 376,      'weight': 335},
    {'category': 'drink',               'name': 'tea',              'color': 'green',           'volume': 877,      'weight': 100},
    {'category': 'drink',               'name': 'water',            'color': 'transparent',     'volume': 400,      'weight': 100},
    {'category': 'snack',               'name': 'cookies',          'color': 'brownish',        'volume': 442,      'weight': 100},
    {'category': 'snack',               'name': 'mentos',           'color': 'blue',            'volume': 235,      'weight': 100},
    {'category': 'snack',               'name': 'pringles',         'color': 'orange',          'volume': 1125,     'weight': 100},
    {'category': 'cleaning_stuff',      'name': 'brush',            'color': 'brown',           'volume': 315,      'weight': 100},
    {'category': 'cleaning_stuff',      'name': 'cloth',            'color': 'yellow',          'volume': 161,      'weight': 100},
    {'category': 'cleaning_stuff',      'name': 'deodorant',        'color': 'pink',            'volume': 314,      'weight': 100},
    {'category': 'cleaning_stuff',      'name': 'hair_gel',         'color': 'transparent',     'volume': 238,      'weight': 100},
    {'category': 'cleaning_stuff',      'name': 'hairspray',        'color': 'blue',            'volume': 490,      'weight': 100},
    {'category': 'cleaning_stuff',      'name': 'sponge',           'color': 'yellow',          'volume': 234,      'weight': 100},
    {'category': 'cleaning_stuff',      'name': 'toothpaste',       'color': 'blue',            'volume': 106,      'weight': 100},
    {'category': 'cleaning_stuff',      'name': 'towel',            'color': 'pink',            'volume': 243,      'weight': 100},
    {'category': 'cutlery',             'name': 'fork',             'color': 'brownish',        'volume': 4,        'weight': 100},
    {'category': 'cutlery',             'name': 'spoon',            'color': 'brownish',        'volume': 4,        'weight': 100},
    {'category': 'cutlery',             'name': 'knife',            'color': 'brownish',        'volume': 5,        'weight': 100},
    {'category': 'cutlery',             'name': 'chopsticks',       'color': 'brownish',        'volume': 20,       'weight': 100}
]

object_names = list(set([ o["name"] for o in objects ]))
object_categories = list(set([ o["category"] for o in objects ]))
object_color = list(set([ o["color"] for o in objects ]))
object_size = list(set([ o["volume"] for o in objects ]))
object_weight = list(set([ o["weight"] for o in objects ]))
# object_groups = list(set([ o["group"] for o in objects ]))
# object_known_objects = list(set([ o["name"] for o in objects ]))

category_locations = {
    "food": {"bookcase": "on_top_of"},
    "container": {"flight_case": "in"},
    "drink": {"dinner_table": "on_top_of"},
    "cleaning_stuff": {"cabinet": "on_top_of"},
    "cutlery": {"bar": "on_top_of"}
}

inspect_areas = {
    "bookcase": ["shelf1", "shelf2", "shelf3", "shelf4", "shelf5"]
}

inspect_positions = {
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

most_probable_location_in_room_map = {
    'livingroom': 'dinner_table',
    'bedroom': 'nightstand',
    'workshop': 'workbench',
    'kitchen': 'cabinet',
    'hallway': 'hallway_table'
}


def get_location_from_room(room_id):
    if room_id in most_probable_location_in_room_map:
        return most_probable_location_in_room_map[room_id]
    return None

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