# COMMON KNOWLEDGE FILE RGO2015

female_names = ["Anna","Beth","Carmen","Jennifer","Jessica","Kimberly","Kristina","Laura","Mary","Sarah"]
male_names = ["Alfred","Charles","Daniel","James","John","Luis","Paul","Richard","Robert","Steve"]
names = female_names + male_names

# This dict holds all locations
locations = [
    {'room':'livingroom', 'category': 'shelf', 'name':'cupboard','manipulation':'yes'},
    {'room':'livingroom', 'category': 'table',      'name':'couchtable','manipulation':'yes'},
    {'room':'kitchen', 'category': 'shelf',      'name':'cabinet',   'manipulation':'yes'},
    {'room':'livingroom', 'category': 'appliance',  'name':'tv',        'manipulation':'no'},
    {'room':'livingroom', 'category': 'seat',       'name':'couch',     'manipulation':'no'},
    {'room':'kitchen', 'category': '',          'name':'kitchencounter','manipulation':'yes'},
    {'room':'kitchen', 'category': 'appliance', 'name':'stove',         'manipulation':'yes'},
    {'room':'kitchen', 'category': 'appliance', 'name':'fridge',        'manipulation':'yes'},
    {'room':'kitchen', 'category': 'appliance', 'name':'sink',          'manipulation':'yes'},
    {'room':'kitchen', 'category': 'utility',   'name':'trashbin',      'manipulation':'no'},
    {'room':'kitchen', 'category': 'table',     'name':'bartable',      'manipulation':'yes'},
    {'room':'hallway', 'category': 'table',    'name':'small_table',   'manipulation':'yes'},
    {'room':'hallway', 'category': 'utility',  'name':'coathanger',    'manipulation':'no'},
    {'room':'bedroom', 'category': 'seat',  'name':'bed',                   'manipulation':'yes'},
    {'room':'bedroom', 'category': 'table', 'name':'left_bedside_table',    'manipulation':'yes'},
    {'room':'bedroom', 'category': 'table', 'name':'right_bedside_table',   'manipulation':'yes'},
    {'room':'office', 'category': 'table', 'name':'counter',        'manipulation':'yes'},
    {'room':'office', 'category': 'shelf', 'name':'left_bookcase',  'manipulation':'yes'},
    {'room':'office', 'category': 'shelf', 'name':'right_bookcase', 'manipulation':'yes'},
    {'room':'livingroom', 'category': 'table', 'name':'dinnertable',    'manipulation':'yes'},
    {'room':'office', 'category': 'table', 'name':'desk',           'manipulation':'yes'}
]

location_rooms = list(set([ o["room"] for o in locations ]))
location_categories = list(set([ o["category"] for o in locations ]))
location_names = list(set([ o["name"] for o in locations ]))
location_manipulatable = list(set([ o["manipulation"] for o in locations ]))

rooms = location_rooms

objects = [
{'category': 'decoration',  'placement': 'couchtable',  'group': 'known',   'name': 'candle'},
{'category': 'decoration',  'placement': 'couchtable',  'group': 'known',   'name': 'yellow candle'},
{'category': 'food',    'placement': 'stove',       'group': 'known',       'name': 'bubblegum'},
{'category': 'leisure', 'placement': 'small_table', 'group': 'known',       'name': 'cup'},
{'category': 'leisure', 'placement': 'small_table', 'group': 'known',       'name': 'deodorant',          'sub-category': 'medicine'},
{'category': 'leisure', 'placement': 'small_table', 'group': 'known',       'name': 'cigarettes'},
{'category': 'drink',   'placement': 'cabinet',     'group': 'known',       'name': 'beer'},
{'category': 'tool',    'placement': 'counter',     'group': 'known',       'name': 'wd40'},
{'category': 'tool',    'placement': 'counter',     'group': 'known',       'name': 'filler'},
{'category': 'food',    'placement': 'stove',       'group': 'known',       'name': 'chocolate_cereals',  'sub-category': 'cereal'},
{'category': 'drink',   'placement': 'cabinet',     'group': 'known',       'name': 'coke',               'sub-category': 'medicine'},
{'category': 'food',    'placement': 'stove',       'group': 'known',       'name': 'chocosticks'},
{'category': 'food',    'placement': 'stove',       'group': 'known',       'name': 'noodles'},
{'category': 'drink',   'placement': 'cabinet',     'group': 'known',       'name': 'coffee'},
{'category': 'food',    'placement': 'stove',       'group': 'known',       'name': 'cranberry_cereals',  'sub-category': 'cereal'},
{'category': 'food',    'placement': 'stove',       'group': 'known',       'name': 'muesli_cereals',     'sub-category': 'cereal'},
{'category': 'food',    'placement': 'stove',       'group': 'known',       'name': 'bubblemint',         'sub-category': 'medicine'},
{'category': 'tool',    'placement': 'counter',     'group': 'known',       'name': 'brush'},
{'category': 'drink',   'placement': 'cabinet',     'group': 'known',       'name': 'red_bull'},
{'category': 'drink',   'placement': 'cabinet',     'group': 'known',       'name': 'meadow_milk',        'sub-category': 'milk'},
{'category': 'drink',   'placement': 'cabinet',     'group': 'known',       'name': 'ice-tea'},
{'category': 'food',    'placement': 'stove',       'group': 'known',       'name': 'mints',              'sub-category': 'medicine'},
{'category': 'food',    'placement': 'stove',       'group': 'known',       'name': 'pringles'},
{'category': 'drink',   'placement': 'cabinet',     'group': 'known',       'name': 'fresh_milk',         'sub-category': 'milk'},
{'category': 'food',    'placement': 'stove',       'group': 'known',       'name': 'oblates'},
{'category': 'drink',   'placement': 'cabinet',     'group': 'known',       'name': 'coffeepads'},
{'category': 'food',    'placement': 'stove',       'group': 'known',       'name': 'peanut'},
{'category': 'food',    'placement': 'stove',       'group': 'alike',       'name': 'mandarine',          'sub-category': 'fruit'},
{'category': 'tool',    'placement': 'counter',     'group': 'alike',       'name': 'tape'},
{'category': 'food',    'placement': 'stove',       'group': 'alike',       'name': 'lemon',              'sub-category': 'fruit'},
{'category': 'tool',    'placement': 'counter',     'group': 'alike',       'name': 'twine'},
{'category': 'drink',   'placement': 'cabinet',     'group': 'alike',       'name': 'fanta'},
{'category': 'food',    'placement': 'stove',       'group': 'alike',       'name': 'apple',              'sub-category': 'fruit'},
{'category': 'bowl',    'placement': '',            'group': 'containers',  'name': 'bowl'},
{'category': 'tray',    'placement': '',            'group': 'containers',  'name': 'tray'}]

object_names = list(set([ o["name"] for o in objects ]))
object_categories = list(set([ o["category"] for o in objects ]))
object_groups = list(set([ o["group"] for o in objects ]))
object_placements = list(set([ o["placement"] for o in objects ]))
object_known_objects = list(set([ o["name"] for o in objects ]))

category_locations = {
}

inspect_areas = {
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
    for o in objects:
        if o["category"] == obj_cat:
            return (o["placement"], "on_top_of")
    return None

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



