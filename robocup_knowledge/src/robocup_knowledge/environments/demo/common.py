# COMMON KNOWLEDGE FILE RWC2017

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
    if location in location_rooms:
        return location
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


def get_object_category_location(obj_cat):
    # Returns (location, area_name)
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
