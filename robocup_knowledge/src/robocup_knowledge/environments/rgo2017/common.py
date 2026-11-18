# COMMON KNOWLEDGE FILE RGO2017

from __future__ import print_function

female_names = ["emma","olivia","sophia","ava","isabella","mia","abigail","emily","charlotte","harper"]
male_names = ["noah","liam","mason","jacob","william","ethan","james","alexander","michael","benjamin"]
names = female_names + male_names

# This dict holds all locations
locations = [
    { 'name':'bookshelf',       'room':'living_room', 'category': 'shelf',   'manipulation':'yes' },
    { 'name':'sofa',            'room':'living_room', 'category': 'seat',    'manipulation':'no'  },
    { 'name':'couch_table',     'room':'living_room', 'category': 'table',   'manipulation':'yes' },
    { 'name':'side_table',      'room':'living_room', 'category': 'table',   'manipulation':'yes' },
    { 'name':'tv_stand',        'room':'living_room', 'category': 'beacon',  'manipulation':'no'  },

    { 'name':'kitchencounter',  'room':'kitchen',    'category': 'table',   'manipulation':'yes' },
    { 'name':'stove',           'room':'kitchen',    'category': 'table',   'manipulation':'yes' },
    { 'name':'desk',            'room':'kitchen',    'category': 'table',   'manipulation':'yes' },
    { 'name':'bar',             'room':'kitchen',    'category': 'table',   'manipulation':'yes' },

    { 'name':'bed',             'room':'bedroom',    'category': 'seat',    'manipulation':'no'  },
    { 'name':'closet',          'room':'bedroom',    'category': 'shelf',   'manipulation':'yes' },

    { 'name':'dinner_table',    'room':'dining_room', 'category': 'table',   'manipulation':'yes' },
    { 'name':'cabinet',         'room':'dining_room', 'category': 'shelf',   'manipulation':'yes' }

]

location_rooms = list(set([ o["room"] for o in locations ]))
location_categories = list(set([ o["category"] for o in locations ]))
location_names = list(set([ o["name"] for o in locations ]))
manipulation_locations = list(set([ o["name"] for o in locations if o["manipulation"] == "yes" ]))

# hack
most_probable_location_in_room_map = {
    'dining_room': 'dinner_table',
    'bedroom': 'closet',
    'living_room': 'couch_table',
    'kitchen': 'desk'
}


def get_location_from_room(room_id):
    if room_id in most_probable_location_in_room_map:
        return most_probable_location_in_room_map[room_id]
    return None

# rooms = location_rooms + ["workshop"]

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

    "cutlery": {"cabinet": "shelf2"},
    "container": {"bookshelf": "shelf1"},
    "drink": {"kitchencounter": "on_top_of"},
    "snack": {"couch_table": "on_top_of"},
    "food": {"stove": "on_top_of"},
    "cleaning_stuff": {"closet": "on_top_of"},
    "fruit": {"desk": "on_top_of"}
}

inspect_areas = {
    "cabinet": ["shelf1", "shelf2", "shelf3", "shelf4"],
    "bookshelf": ["shelf1", "shelf2", "shelf3", "shelf4"]
}

inspect_positions = {
}

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


class bcolors:
    ''' colors from printing on screen '''
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
        print(prefix + bcolors.OKBLUE + sentence + bcolors.ENDC)

    def printError(sentence):
        print(prefix + bcolors.FAIL + sentence + bcolors.ENDC)

    def printWarning(sentence):
        print(prefix + bcolors.WARNING + sentence + bcolors.ENDC)

    return printOk, printError, printWarning
