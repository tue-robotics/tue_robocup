# COMMON KNOWLEDGE FILE RGO2018

female_names = ["angie", "mary", "amy", "kimberley", "lisa", "melissa", "michelle", "jennifer", "elizabeth", "julie"]
male_names = ["brian","michael","christopher","william","john","david","james","robert","scott","richard"]
names = female_names + male_names

# This dict holds all locations
locations = [

    { 'name':'bed',             'room':'bedroom',    'category': 'beacon',  'manipulation':'no'  },
    { 'name':'desk',      	    'room':'bedroom',    'category': 'shelf',   'manipulation':'yes' },
    { 'name':'bookcase',   	    'room':'bedroom',    'category': 'shelf',   'manipulation':'yes' },
    { 'name':'side_table',      'room':'bedroom',    'category': 'table',   'manipulation':'yes' },

    { 'name':'bar', 		    'room':'kitchen',    'category': 'bar',     'manipulation':'yes' },
    { 'name':'kitchen_table',   'room':'kitchen',    'category': 'table',   'manipulation':'yes' },
    { 'name':'kitchen_cabinet', 'room':'kitchen',    'category': 'table',   'manipulation':'yes' },
    { 'name':'sink',            'room':'kitchen',    'category': 'table',   'manipulation':'no'  },

    { 'name':'tv_table',       	'room':'living_room', 'category': 'table',  'manipulation':'yes' },
    { 'name':'cupboard',        'room':'living_room', 'category': 'shelf',  'manipulation':'yes' },
    { 'name':'couch',           'room':'living_room', 'category': 'seat',   'manipulation':'yes' },
    { 'name':'couch_table',     'room':'living_room', 'category': 'beacon', 'manipulation':'yes' },

    { 'name':'dining_table',    'room':'dining_room', 'category': 'beacon', 'manipulation':'yes' },
    { 'name':'cabinet',         'room':'dining_room', 'category': 'shelf',  'manipulation':'yes' },
    { 'name':'display_case',    'room':'dining_room', 'category': 'table',  'manipulation':'no'  },
    { 'name':'storage_shelf',   'room':'dining_room', 'category': 'shelf',  'manipulation':'yes' }

]

location_rooms = list(set([ o["room"] for o in locations ]))
location_categories = list(set([ o["category"] for o in locations ]))
location_names = list(set([ o["name"] for o in locations ]))
manipulation_locations = list(set([ o["name"] for o in locations if o["manipulation"] == "yes" ]))

rooms = location_rooms + ["workshop"]

objects = [
    {'category': 'care',                'name': 'shower_gel',        'color': 'pink'  },
    {'category': 'care',                'name': 'soap',              'color': 'white' },
    {'category': 'care',                'name': 'toothpaste',        'color': 'green' },
    {'category': 'cleaning_stuff',      'name': 'sponge',            'color': 'yellow'},
    {'category': 'cleaning_stuff',      'name': 'wiper',             'color': 'yellow'},
    {'category': 'container',   	    'name': 'box',               'color': 'yellow'},
    {'category': 'container',   	    'name': 'tray',              'color': 'white' },
    {'category': 'drink',   	  	    'name': 'cacao',             'color': 'brown' },
    {'category': 'drink',   	  	    'name': 'coke',              'color': 'red'   },
    {'category': 'drink',   	  	    'name': 'malz',              'color': 'brown' },
    {'category': 'drink',   	  	    'name': 'mixdrink',          'color': 'brown' },
    {'category': 'drink',   	  	    'name': 'orange_juice',      'color': 'orange'},
    {'category': 'drink',   	  	    'name': 'perppermint_tea',   'color': 'green' },
    {'category': 'drink',   	  	    'name': 'water',      	     'color': 'transparent'},
    {'category': 'snack',   	  	    'name': 'cookies',           'color': 'blue'  },
    {'category': 'snack',   	  	    'name': 'fruit_bar',         'color': 'green' },
    {'category': 'snack',   	  	    'name': 'kinder',            'color': 'white' },
    {'category': 'snack',   	  	    'name': 'nuts',              'color': 'yellow'},
    {'category': 'food',                'name': 'apple',             'color': 'green' },
    {'category': 'food',                'name': 'green_paprika',     'color': 'green' },
    {'category': 'food',                'name': 'kiwi',              'color': 'brown' },
    {'category': 'food',                'name': 'lemon',             'color': 'yellow'},
    {'category': 'food',                'name': 'noodles',           'color': 'yellow'},
    {'category': 'food',                'name': 'pepper',            'color': 'brown' },
    {'category': 'food',                'name': 'salt',              'color': 'salt'  },
    {'category': 'food',                'name': 'tomato',            'color': 'red'   },
    {'category': 'help_me_carry',       'name': 'bag',               'color': 'red'   },
    {'category': 'dishwasher_test',     'name': 'dishwasher_tray',   'color': 'white' },
]

object_names = list(set([ o["name"] for o in objects ]))
object_categories = list(set([ o["category"] for o in objects ]))
object_color = list(set([ o["color"] for o in objects ]))
# object_groups = list(set([ o["group"] for o in objects ]))
# object_known_objects = list(set([ o["name"] for o in objects ]))

category_locations = {
#To do! Fix in from bookcase and cabinet
    "care": {"bookcase": ""},
    "container": {"cupboard": "on_top_off"},
    "drink": {"kitchen_table": "on_top_of"},
    "snack": {"couch_table": "on_top_of"},
    "food": {"cabinet": ""},
    "cleaning_stuff": {"closet": "on_top_of"}
}

inspect_areas = {
#To do!
    "cabinet": ["shelf1", "shelf2", "shelf3", "shelf4"],
    "bookcase": ["shelf1", "shelf2", "shelf3", "shelf4"]
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



