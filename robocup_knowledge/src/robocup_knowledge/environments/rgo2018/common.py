# COMMON KNOWLEDGE FILE RGO2018

from __future__ import print_function

female_names = ["angie", "mary", "amy", "kimberley", "lisa", "melissa", "michelle", "jennifer", "elizabeth", "julie"]
male_names = ["brian", "michael", "christopher", "william", "john", "david", "james", "robert", "scott", "richard"]
names = female_names + male_names

# This dict holds all locations
locations = [

    {'name': 'bed',             'room': 'bedroom',    'category': 'beacon',  'manipulation': 'no'},
    {'name': 'desk',            'room': 'bedroom',    'category': 'shelf',   'manipulation': 'yes'},
    {'name': 'bookcase',        'room': 'bedroom',    'category': 'shelf',   'manipulation': 'yes'},
    {'name': 'side_table',      'room': 'bedroom',    'category': 'table',   'manipulation': 'yes'},

    {'name': 'bar',             'room': 'kitchen',    'category': 'bar',     'manipulation': 'yes'},
    {'name': 'kitchen_table',   'room': 'kitchen',    'category': 'table',   'manipulation': 'yes'},
    {'name': 'kitchen_cabinet', 'room': 'kitchen',    'category': 'table',   'manipulation': 'yes'},
    {'name': 'sink',            'room': 'kitchen',    'category': 'table',   'manipulation': 'no'},

    {'name': 'tv_table',        'room': 'living_room', 'category': 'table',  'manipulation': 'yes'},
    {'name': 'cupboard',        'room': 'living_room', 'category': 'shelf',  'manipulation': 'yes'},
    {'name': 'couch',           'room': 'living_room', 'category': 'seat',   'manipulation': 'yes'},
    {'name': 'couch_table',     'room': 'living_room', 'category': 'beacon', 'manipulation': 'yes'},

    {'name': 'dining_table',    'room': 'dining_room', 'category': 'beacon', 'manipulation': 'yes'},
    {'name': 'cabinet',         'room': 'dining_room', 'category': 'shelf',  'manipulation': 'yes'},
    {'name': 'display_case',    'room': 'dining_room', 'category': 'table',  'manipulation': 'no'},
    {'name': 'storage_shelf',   'room': 'dining_room', 'category': 'shelf',  'manipulation': 'yes'}

]

location_rooms = list(set([ o["room"] for o in locations ]))
location_categories = list(set([ o["category"] for o in locations ]))
location_names = list(set([ o["name"] for o in locations ]))
manipulation_locations = list(set([ o["name"] for o in locations if o["manipulation"] == "yes" ]))

rooms = location_rooms + ["workshop"]

objects = [
    {'category': 'care',                'name': 'shower_gel',        'color': 'pink'},
    {'category': 'care',                'name': 'soap',              'color': 'white'},
    {'category': 'care',                'name': 'toothpaste',        'color': 'green'},
    {'category': 'cleaning_stuff',      'name': 'sponge',            'color': 'yellow'},
    {'category': 'cleaning_stuff',      'name': 'wiper',             'color': 'yellow'},
    {'category': 'container',   	    'name': 'box',               'color': 'yellow'},
    {'category': 'container',   	    'name': 'tray',              'color': 'white'},
    {'category': 'drinks',   	  	    'name': 'beer',              'color': 'yellow'},
    {'category': 'drinks',   	  	    'name': 'cacao',             'color': 'brown'},
    {'category': 'drinks',   	  	    'name': 'coke',              'color': 'red'},
    {'category': 'drinks',   	  	    'name': 'malz',              'color': 'brown'},
    {'category': 'drinks',   	  	    'name': 'mixdrink',          'color': 'brown'},
    {'category': 'drinks',   	  	    'name': 'orange_juice',      'color': 'orange'},
    {'category': 'drinks',   	  	    'name': 'peppermint_tea',    'color': 'green'},
    {'category': 'drinks',   	  	    'name': 'water',      	     'color': 'transparent'},
    {'category': 'snacks',   	  	    'name': 'cookies',           'color': 'blue'},
    {'category': 'snacks',   	  	    'name': 'fruit_bar',         'color': 'green'},
    {'category': 'snacks',   	  	    'name': 'kinder',            'color': 'white'},
    {'category': 'snacks',   	  	    'name': 'nuts',              'color': 'yellow'},
    {'category': 'food',                'name': 'apple',             'color': 'green'},
    {'category': 'food',                'name': 'green_paprika',     'color': 'green'},
    {'category': 'food',                'name': 'kiwi',              'color': 'brown'},
    {'category': 'food',                'name': 'lemon',             'color': 'yellow'},
    {'category': 'food',                'name': 'noodles',           'color': 'yellow'},
    {'category': 'food',                'name': 'pepper',            'color': 'brown'},
    {'category': 'food',                'name': 'salt',              'color': 'salt'},
    {'category': 'food',                'name': 'tomato',            'color': 'red'},
    {'category': 'kitchen_stuff',       'name': 'fork',              'color': 'green'},
    {'category': 'kitchen_stuff',       'name': 'spoon',             'color': 'green'},
    {'category': 'kitchen_stuff',       'name': 'knife',             'color': 'green'},
    {'category': 'kitchen_stuff',       'name': 'plate',             'color': 'green'},
    {'category': 'kitchen_stuff',       'name': 'cup',               'color': 'green'}


]

# {'category': 'help_me_carry', 'name': 'bag', 'color': 'red'},
# {'category': 'dishwasher_test', 'name': 'dishwasher_tray', 'color': 'white'},

object_names = list(set([ o["name"] for o in objects ]))
object_categories = list(set([ o["category"] for o in objects ]))
object_color = list(set([ o["color"] for o in objects ]))
# object_groups = list(set([ o["group"] for o in objects ]))
# object_known_objects = list(set([ o["name"] for o in objects ]))

category_locations = {
#Test
    "care": {"bookcase": "shelf2"},
    "container": {"cupboard": "on_top_off"},
    "drinks": {"kitchen_table": "on_top_of"},
    "snacks": {"couch_table": "on_top_of"},
    "food": {"cabinet": "shelf2"},
    "cleaning_stuff": {"kitchen_cabinet": "on_top_of"},
    "kitchen_stuff" : {"kitchen_table": "on_top_of"}
}

inspect_areas = {
#Test
    "cabinet": ["shelf2", "shelf3", "shelf4"],
    "display_case": ["shelf2", "shelf3", "shelf4"],
    "storage_shelf": ["shelf2", "shelf3", "shelf4"],
    "bookcase": ["shelf2", "shelf3", "shelf4"]
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
        print(prefix + bcolors.OKBLUE + sentence + bcolors.ENDC)

    def printError(sentence):
        print(prefix + bcolors.FAIL + sentence + bcolors.ENDC)

    def printWarning(sentence):
        print(prefix + bcolors.WARNING + sentence + bcolors.ENDC)

    return printOk, printError, printWarning
