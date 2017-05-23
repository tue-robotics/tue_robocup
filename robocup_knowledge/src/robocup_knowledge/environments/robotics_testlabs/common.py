# COMMON KNOWLEDGE FILE ROBOTICS TESTLABS

female_names = ["anna","beth","carmen","jennifer","jessica","kimberly","kristina","laura","mary","sarah"]
male_names = ["alfred","charles","daniel","james","john","luis","paul","richard","robert","steve"]
names = female_names + male_names

# This dict holds all locations
locations = [
    { 'name':'couch_table',   'room':'livingroom', 'category': 'table',   'manipulation':'yes' },
    { 'name':'dinner_table',  'room':'livingroom', 'category': 'table',   'manipulation':'yes' },
    { 'name':'bar',           'room':'livingroom', 'category': 'table',   'manipulation':'yes' },
    { 'name':'bookcase',      'room':'livingroom', 'category': 'shelf',   'manipulation':'yes' },

    { 'name':'cabinet',       'room':'kitchen',    'category': 'shelf',   'manipulation':'yes' },
    { 'name':'trashbin',      'room':'kitchen',    'category': 'utility', 'manipulation':'no'  },
    { 'name':'plant',         'room':'kitchen',    'category': 'plant',   'manipulation':'no'  },

    { 'name':'bed',           'room':'bedroom',    'category': 'seat',    'manipulation':'yes' },
    { 'name':'nightstand',    'room':'bedroom',    'category': 'table',   'manipulation':'yes' },

    { 'name':'flight_case',   'room':'workshop',   'category': 'table',   'manipulation':'no'  },
    { 'name':'battery_table', 'room':'workshop',   'category': 'table',   'manipulation':'no' },
    { 'name':'workbench',     'room':'workshop',   'category': 'table',   'manipulation':'yes' },

    { 'name':'hallway_table', 'room':'hallway',    'category': 'table',   'manipulation':'yes' }
]

location_rooms = list(set([ o["room"] for o in locations ]))
location_categories = list(set([ o["category"] for o in locations ]))
location_names = list(set([ o["name"] for o in locations ]))
manipulation_locations = list(set([ o["name"] for o in locations if o["manipulation"] == "yes" ]))

rooms = location_rooms + ["workshop"]

objects = [
    {'category': 'food',                'name': 'apple'             },
    {'category': 'food',                'name': 'bread'             },
    {'category': 'food',                'name': 'cereals'           },
    {'category': 'food',                'name': 'cornflakes'        },
    {'category': 'food',                'name': 'crackers'          },
    {'category': 'food',                'name': 'lemon'             },
    {'category': 'food',                'name': 'noodles'           },
    {'category': 'food',                'name': 'paprika'           },
    {'category': 'food',                'name': 'peas'              },
    {'category': 'food',                'name': 'pepper'            },
    {'category': 'food',                'name': 'potato'            },
    {'category': 'food',                'name': 'potato_soup'       },
    {'category': 'food',                'name': 'salt'              },
    {'category': 'food',                'name': 'tomato_pasta'      },
    {'category': 'container',           'name': 'bag'               },
    {'category': 'container',           'name': 'basket'            },
    {'category': 'container',           'name': 'coffecup'          },
    {'category': 'container',           'name': 'plate'             },
    {'category': 'container',           'name': 'red_bowl'          },
    {'category': 'container',           'name': 'white_bowl'        },
    {'category': 'drink',               'name': 'banana_milk'       },
    {'category': 'drink',               'name': 'cappucino'         },
    {'category': 'drink',               'name': 'coke'              },
    {'category': 'drink',               'name': 'orange_drink'      },
    {'category': 'drink',               'name': 'water'             },
    {'category': 'snack',               'name': 'chocolate_cookies' },
    {'category': 'snack',               'name': 'egg'               },
    {'category': 'snack',               'name': 'party_cracker'     },
    {'category': 'snack',               'name': 'pringles'          },
    {'category': 'cleaning_stuff',      'name': 'cloth'             },
    {'category': 'cleaning_stuff',      'name': 'paper'             },
    {'category': 'cleaning_stuff',      'name': 'sponge'            },
    {'category': 'cleaning_stuff',      'name': 'towel'             },
    {'category': 'cutlery',             'name': 'fork'              },
    {'category': 'cutlery',             'name': 'spoon'             },
    {'category': 'cutlery',             'name': 'knife'             }
]

object_names = list(set([ o["name"] for o in objects ]))
object_categories = list(set([ o["category"] for o in objects ]))
# object_groups = list(set([ o["group"] for o in objects ]))
# object_known_objects = list(set([ o["name"] for o in objects ]))

category_locations = {
    "food": {"bookcase": "shelf3"},
    "drink": {"cabinet": "on_top_of"},
    "tool": {"workbench": "on_top_of"},
    "decoration": {"dinner_table": "on_top_of"},
    "leisure": {"bar": "on_top_of"},
    "bowl": {"dinner_table": "on_top_of"},
    "tray": {"dinner_table": "on_top_of"}
}

inspect_areas = {
    "bookcase" : ["shelf1", "shelf2", "shelf3", "shelf4", "shelf5"]
}

inspect_positions = {
}

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
	name: name of the progam that instanciates make_prints
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



