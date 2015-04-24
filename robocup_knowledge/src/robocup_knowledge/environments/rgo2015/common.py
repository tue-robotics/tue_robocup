# COMMON KNOWLEDGE FILE RGO2015

female_names = ["Anna","Beth","Carmen","Jennifer","Jessica","Kimberly","Kristina","Laura","Mary","Sarah"]
male_names = ["Alfred","Charles","Daniel","James","John","Luis","Paul","Richard","Robert","Steve"]
names = female_names + male_names

# NEXT TIME: locations should be rooms
locations = [ 'kitchen', 'livingroom', 'hall', 'bedroom', 'office']

# This dict holds all locations
# fill in: [{'room':'', 'category': '', 'location_name':'', 'manipulation':''}]
all_locations = [{'room':'livingroom', 'category': 'shelf', 'location_name':'cupboard','manipulation':'yes'},
{'room':'livingroom', 'category': 'table',      'location_name':'couchtable','manipulation':'yes'},
{'room':'livingroom', 'category': 'shelf',      'location_name':'cabinet',   'manipulation':'yes'},
{'room':'livingroom', 'category': 'appliance',  'location_name':'tv',        'manipulation':'no'},
{'room':'livingroom', 'category': 'seat',       'location_name':'couch',     'manipulation':'no'},
{'room':'kitchen', 'category': '',          'location_name':'kitchencounter','manipulation':'yes'},
{'room':'kitchen', 'category': 'appliance', 'location_name':'stove',         'manipulation':'yes'},
{'room':'kitchen', 'category': 'appliance', 'location_name':'fridge',        'manipulation':'yes'},
{'room':'kitchen', 'category': 'appliance', 'location_name':'sink',          'manipulation':'yes'},
{'room':'kitchen', 'category': 'utility',   'location_name':'trashbin',      'manipulation':'no'},
{'room':'kitchen', 'category': 'table',     'location_name':'bartable',      'manipulation':'yes'},
{'room':'hall', 'category': 'table',    'location_name':'small_table',   'manipulation':'yes'},
{'room':'hall', 'category': 'utility',  'location_name':'coathanger',    'manipulation':'no'},
{'room':'bedroom', 'category': 'seat',  'location_name':'bed',                   'manipulation':'yes'},
{'room':'bedroom', 'category': 'table', 'location_name':'left_bedside_table',    'manipulation':'yes'},
{'room':'bedroom', 'category': 'table', 'location_name':'right_bedside_table',   'manipulation':'yes'},
{'room':'office', 'category': 'table', 'location_name':'counter',        'manipulation':'yes'},
{'room':'office', 'category': 'shelf', 'location_name':'left_bookcase',  'manipulation':'yes'},
{'room':'office', 'category': 'shelf', 'location_name':'right_bookcase', 'manipulation':'yes'},
{'room':'office', 'category': 'table', 'location_name':'dinnertable',    'manipulation':'yes'},
{'room':'office', 'category': 'table', 'location_name':'desk',           'manipulation':'yes'}]

location_rooms = list(set([ o["room"] for o in all_locations ]))
location_categories = list(set([ o["category"] for o in all_locations ]))
location_names = list(set([ o["location_name"] for o in all_locations ]))
location_manipulatable = list(set([ o["manipulation"] for o in all_locations ]))

objects = [{'category': 'food', 'placement': 'stove', 'group': 'known', 'name': 'bubblegum'}, 
{'category': 'decoration', 'placement': 'couchtable', 'group': 'known', 'name': 'candle'}, 
{'category': 'leisure', 'placement': 'small_table', 'group': 'known', 'name': 'cup'}, 
{'category': 'drink',   'placement': 'cabinet',     'group': 'known', 'name': 'beer'},
{'category': 'tool',    'placement': 'counter',     'group': 'known', 'name': 'wd40'}, 
{'category': 'tool',    'placement': 'counter',     'group': 'known', 'name': 'filler'},
{'category': 'food',    'sub-category': 'cereal',   'placement': 'stove',       'group': 'known', 'name': 'chocolate_cereals'},
{'category': 'drink',   'placement': 'cabinet',     'group': 'known', 'name': 'coke'},
{'category': 'food',    'placement': 'stove',       'group': 'known', 'name': 'chocosticks'},
{'category': 'decoration', 'placement': 'couchtable', 'group': 'known', 'name': 'yellow candle'},
{'category': 'food',    'placement': 'stove',       'group': 'known', 'name': 'noodles'}, 
{'category': 'drink',   'placement': 'cabinet',     'group': 'known', 'name': 'coffee'},
{'category': 'food',    'sub-category': 'cereal',   'placement': 'stove',   'group': 'known', 'name': 'cranberry_cereals'},
{'category': 'food',    'sub-category': 'cereal',   'placement': 'stove',       'group': 'known', 'name': 'muesli_cereals'}, 
{'category': 'food',    'placement': 'stove',       'group': 'known', 'name': 'bubblemint'}, 
{'category': 'leisure', 'placement': 'small_table', 'group': 'known', 'name': 'deodorant'}, 
{'category': 'tool',    'placement': 'counter',     'group': 'known', 'name': 'brush'}, 
{'category': 'leisure', 'placement': 'small_table', 'group': 'known', 'name': 'cigarettes'}, 
{'category': 'drink',   'placement': 'cabinet',     'group': 'known', 'name': 'red_bull'}, 
{'category': 'drink',   'sub-category': 'milk',     'placement': 'cabinet',     'group': 'known', 'name': 'meadow_milk'}, 
{'category': 'drink',   'placement': 'cabinet',     'group': 'known', 'name': 'ice-tea'}, 
{'category': 'food',    'placement': 'stove',       'group': 'known', 'name': 'mints'}, 
{'category': 'food',    'placement': 'stove',       'group': 'known', 'name': 'pringles'}, 
{'category': 'drink',   'sub-category': 'milk',     'placement': 'cabinet',     'group': 'known', 'name': 'fresh_milk'}, 
{'category': 'food',    'placement': 'stove',       'group': 'known', 'name': 'oblates'}, 
{'category': 'drink',   'placement': 'cabinet',     'group': 'known', 'name': 'coffeepads'}, 
{'category': 'food',    'placement': 'stove',       'group': 'known', 'name': 'peanut'}, 
{'category': 'food',    'sub-category': 'fruit',    'placement': 'stove',       'group': 'alike', 'name': 'mandarine'}, 
{'category': 'tool',    'placement': 'counter',     'group': 'alike', 'name': 'tape'}, 
{'category': 'food',    'sub-category': 'fruit',    'placement': 'stove',       'group': 'alike', 'name': 'lemon'}, 
{'category': 'tool',    'placement': 'counter',     'group': 'alike', 'name': 'twine'}, 
{'category': 'drink',   'placement': 'cabinet',     'group': 'alike', 'name': 'fanta'}, 
{'category': 'food',    'sub-category': 'fruit',    'placement': 'stove',       'group': 'alike', 'name': 'apple'}, 
{'category': 'bowl',    'placement': '',            'group': 'containers', 'name': 'bowl'}, 
{'category': 'tray',    'placement': '',            'group': 'containers', 'name': 'tray'}]

object_names = list(set([ o["name"] for o in objects ]))
object_categories = list(set([ o["category"] for o in objects ]))
object_groups = list(set([ o["group"] for o in objects ]))
object_placements = list(set([ o["placement"] for o in objects ]))
object_known_objects = list(set([ o["name"] for o in objects ]))


