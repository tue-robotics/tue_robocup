# COMMON KNOWLEDGE FILE RWC2017
from __future__ import print_function

female_names = ["emma", "olivia", "sophia", "ava", "isabella", "mia", "abigail", "emily", "charlotte", "harper"]
male_names = ["noah", "liam", "mason", "jacob", "william", "ethan", "james", "alexander", "michael", "benjamin"]
names = female_names + male_names

# This dict holds all locations
# ToDo: check with Loy: category
locations = [
    {"name": "amigo_case", "room": "ToDo", "category": "ToDo", "manipulation": "no"},
    {"name": "laptop_case", "room": "ToDo", "category": "ToDo", "manipulation": "no"},
    {"name": "amigo_case_lid", "room": "ToDo", "category": "ToDo", "manipulation": "yes"},
    {"name": "laptop_case_lid", "room": "ToDo", "category": "ToDo", "manipulation": "yes"},
]

location_rooms = list(set([o["room"] for o in locations]))
location_categories = list(set([o["category"] for o in locations]))
location_names = list(set([o["name"] for o in locations]))
manipulation_locations = list(set([o["name"] for o in locations if o["manipulation"] == "yes"]))

objects = [
    {"name": "coke", "category": "drink"},
    {"name": "fanta", "category": "drink"}
]

object_names = list(set([o["name"] for o in objects]))
object_categories = list(set([o["category"] for o in objects]))

category_locations = {
    "drink": {"amigo_case_lid": "on_top_of"}
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
