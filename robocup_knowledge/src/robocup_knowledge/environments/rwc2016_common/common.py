# COMMON KNOWLEDGE FILE RWC2016A
from __future__ import print_function

female_names = ["Emma", "Taylor", "Sophia", "Isabella", "Ava", "Robin", "Emily", "Angel", "Madison", "Charlotte"]
male_names = ["Noah", "Liam", "Mason", "Jacob", "William", "Ethan", "Michael", "Alexander", "James", "Daniel"]
names = female_names + male_names

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

locations = []
category_locations = {}
inspect_areas = {}
inspect_positions = {}
rooms = []
grab_locations = []
put_locations = []

objects = [
    { 'name' : 'choco_syrup',   'category' : 'candies' },
    { 'name' : 'biscuits',      'category' : 'candies' },
    { 'name' : 'baby_sweets',   'category' : 'candies' },
    { 'name' : 'egg',           'category' : 'candies' },
    { 'name' : 'chips',         'category' : 'snacks' },
    { 'name' : 'pretzels',      'category' : 'snacks' },
    { 'name' : 'pringles',      'category' : 'snacks' },
    { 'name' : 'beer',          'category' : 'drinks' },
    { 'name' : 'coconut_milk',  'category' : 'drinks' },
    { 'name' : 'coke',          'category' : 'drinks' },
    { 'name' : 'tea',           'category' : 'drinks' },
    { 'name' : 'apple',         'category' : 'food' },
    { 'name' : 'paprika',       'category' : 'food' },
    { 'name' : 'pumper_nickel', 'category' : 'food' },
    { 'name' : 'shampoo',       'category' : 'toiletries' },
    { 'name' : 'soap',          'category' : 'toiletries' },
    { 'name' : 'sponge',        'category' : 'toiletries' },
    { 'name' : 'cloth',         'category' : 'toiletries' },
    { 'name' : 'bowl',          'category' : 'containers' },
    { 'name' : 'tray',          'category' : 'containers' },
    { 'name' : 'plate',         'category' : 'containers' }
]

object_names = list(set([ o["name"] for o in objects ]))
object_categories = list(set([ o["category"] for o in objects ]))
