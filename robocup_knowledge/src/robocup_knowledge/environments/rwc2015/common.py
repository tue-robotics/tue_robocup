# COMMON KNOWLEDGE FILE RWC2015

female_names = ["Alex","Angel","Eve","Jamie","Jane","Liza","Melissa","Tracy","Robin","Sophia"] 
male_names = ["Alex","Angel","Edward","Homer","Jamie","John","Kevin","Kurt","Tracy","Robin"] 

names = female_names + male_names

# Previously 'rooms' was called 'locations'
rooms = [ 'kitchen', 'livingroom', 'bedroom', 'hallway']

# This dict holds all locations
# fill in: [{'room':'', 'category': '', 'location_name':'', 'manipulation':''}]
all_locations = [
{'room':'kitchen', 'category': 'appliance', 'location_name':'fridge',		  'manipulation':'no'},
{'room':'kitchen', 'category': 'table', 'location_name':'kitchentable',       'manipulation':'yes'},
{'room':'kitchen', 'category': 'table', 'location_name':'kitchencounter',     'manipulation':'yes'},
{'room':'kitchen', 'category': 'shelf', 'location_name':'cupboard',           'manipulation':'yes'},
{'room':'kitchen', 'category': 'utility',   'location_name':'trashbin',       'manipulation':'no'},
{'room':'livingroom', 'category': 'shelf', 'location_name':'bar',             'manipulation':'yes'},
{'room':'livingroom', 'category': 'table',      'location_name':'couchtable', 'manipulation':'yes'},
{'room':'livingroom', 'category': 'table',      'location_name':'dinnertable','manipulation':'yes'}, # dinnertable has two spots
{'room':'livingroom', 'category': 'seat',       'location_name':'sofa',       'manipulation':'yes'}, # sofa has two spots
{'room':'bedroom', 'category': 'table', 'location_name':'left_bedside_table', 'manipulation':'yes'},
{'room':'bedroom', 'category': 'table', 'location_name':'right_bedside_table','manipulation':'yes'},
{'room':'bedroom', 'category': 'table',  'location_name':'desk',              'manipulation':'yes'},
{'room':'bedroom', 'category': 'seat',  'location_name':'bed',                'manipulation':'yes'}, # bed has two spots
{'room':'hallway', 'category': 'shelf', 'location_name':'bookcase',  		  'manipulation':'yes'}, # bookcase has four spots
{'room':'hallway', 'category': 'table', 'location_name':'hallwaytable', 	  'manipulation':'yes'}] # hallwaytable has two spots

location_rooms = list(set([ o["room"] for o in all_locations ]))
location_categories = list(set([ o["category"] for o in all_locations ]))
location_names = list(set([ o["location_name"] for o in all_locations ]))
location_manipulatable = list(set([ o["manipulation"] for o in all_locations ]))

objects = [
{'category': 'cleaning_stuff', 'placement': 'bookcase', 'group': 'known', 'name': 'sponge'}, 
{'category': 'cleaning_stuff', 'placement': 'bookcase', 'group': 'known', 'name': 'toilet_paper'}, 
{'category': 'cleaning_stuff', 'placement': 'bookcase', 'group': 'known', 'name': 'soap'}, 
{'category': 'cleaning_stuff', 'placement': 'bookcase', 'group': 'known', 'name': 'lotion'},
{'category': 'cleaning_stuff', 'placement': 'bookcase', 'group': 'known', 'name': 'toothpaste'}, 
{'category': 'cleaning_stuff', 'placement': 'bookcase', 'group': 'known', 'name': 'cloth'},
{'category': 'drinks', 'placement': 'kitchentable', 'group': 'known', 'name': 'green_tea'},
{'category': 'drinks', 'placement': 'kitchentable', 'group': 'known', 'name': 'papaya_milk'},
{'category': 'drinks', 'placement': 'kitchentable', 'group': 'known', 'name': 'pure_milk'},
{'category': 'drinks', 'placement': 'kitchentable', 'group': 'known', 'name': 'water'}, 
{'category': 'drinks', 'placement': 'kitchentable', 'group': 'known', 'name': 'orange_juice'},
{'category': 'drinks', 'placement': 'kitchentable', 'group': 'known', 'name': 'beer'},
{'category': 'snacks', 'placement': 'desk', 'group': 'known', 'name': 'biscuits'}, 
{'category': 'snacks', 'placement': 'desk', 'group': 'known', 'name': 'bubble_gum'}, 
{'category': 'snacks', 'placement': 'desk', 'group': 'known', 'name': 'chocolates'},
{'category': 'snacks', 'placement': 'desk', 'group': 'known', 'name': 'tomato_chips'}, 
{'category': 'snacks', 'placement': 'desk', 'group': 'known', 'name': 'barbecue_chips'}, 
{'category': 'snacks', 'placement': 'desk', 'group': 'known', 'name': 'honey_chips'}, 
{'category': 'food', 'placement': 'kitchencounter', 'group': 'known', 'name': 'coconut_cereals'}, 
{'category': 'food', 'placement': 'kitchencounter', 'group': 'known', 'name': 'coco_balls'}, 
{'category': 'food', 'placement': 'kitchencounter', 'group': 'known', 'name': 'egg_stars'},
{'category': 'food', 'placement': 'kitchencounter', 'group': 'known', 'name': 'gram_soup'}, 
{'category': 'food', 'placement': 'kitchencounter', 'group': 'known', 'name': 'bean_sauce'}, 
{'category': 'food', 'placement': 'kitchencounter', 'group': 'known', 'name': 'apple'}, 
{'category': 'food', 'placement': 'kitchencounter', 'group': 'known', 'name': 'lemon'}, 
{'category': 'food', 'placement': 'kitchencounter', 'group': 'known', 'name': 'pear'}, 
{'category': 'container', 'placement': '', 'group': 'containers', 'name': 'bowl'}, 
{'category': 'container', 'placement': '', 'group': 'containers', 'name': 'plate'}, 
{'category': 'container', 'placement': '', 'group': 'containers', 'name': 'tray'}]

object_names = list(set([ o["name"] for o in objects ]))
object_categories = list(set([ o["category"] for o in objects ]))
object_groups = list(set([ o["group"] for o in objects ]))
object_placements = list(set([ o["placement"] for o in objects ]))
object_known_objects = list(set([ o["name"] for o in objects ]))
