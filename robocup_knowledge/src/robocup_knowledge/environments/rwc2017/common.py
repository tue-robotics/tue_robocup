# COMMON KNOWLEDGE FILE RWC2017

female_names = ["emma", "olivia", "sophia", "ava", "isabella", "mia", "abigail", "emily", "charlotte", "harper"]
male_names = ["noah", "liam", "mason", "jacob", "william", "ethan", "james", "alexander", "michael", "benjamin"]
names = female_names + male_names

# This dict holds all locations
# ToDo: check with Loy: category
locations = [
    {"name": "desk", "room": "living_room", "category": "ToDo", "manipulation": "yes"},
    {"name": "left_rack", "room": "living_room", "category": "ToDo", "manipulation": "yes"},
    {"name": "right_rack", "room": "living_room", "category": "ToDo", "manipulation": "yes"},
    {"name": "sideboard", "room": "kitchen", "category": "ToDo", "manipulation": "yes"},
    {"name": "kitchen_table", "room": "kitchen", "category": "ToDo", "manipulation": "yes"},
    {"name": "little_desk", "room": "bedroom", "category": "ToDo", "manipulation": "yes"},
    {"name": "teepee", "room": "bedroom", "category": "ToDo", "manipulation": "no"},
    {"name": "bed", "room": "bedroom", "category": "ToDo", "manipulation": "yes"},
    {"name": "entrance_shelf", "room": "entrance", "category": "ToDo", "manipulation": "yes"},
    {"name": "kitchen_shelf", "room": "kitchen", "category": "ToDo", "manipulation": "yes"},
    {"name": "bookcase", "room": "living_room", "category": "ToDo", "manipulation": "yes"},
    {"name": "sofa", "room": "living_room", "category": "ToDo", "manipulation": "yes"},
    {"name": "coffee_table", "room": "living_room", "category": "ToDo", "manipulation": "yes"},
    {"name": "tv", "room": "living_room", "category": "ToDo", "manipulation": "no"},
    {"name": "bistro_table", "room": "balcony", "category": "ToDo", "manipulation": "yes"},
    {"name": "left_planks", "room": "balcony", "category": "ToDo", "manipulation": "yes"},
    {"name": "right_planks", "room": "balcony", "category": "ToDo", "manipulation": "yes"},
    {"name": "balcony_shelf", "room": "balcony", "category": "ToDo", "manipulation": "yes"},
    {"name": "kitchen_counter", "room": "kitchen", "category": "ToDo", "manipulation": "yes"},
    {"name": "fridge", "room": "kitchen", "category": "ToDo", "manipulation": "yes"},
    {"name": "kitchen_rack", "room": "kitchen", "category": "ToDo", "manipulation": "yes"}
]

location_rooms = list(set([o["room"] for o in locations])) + ['corridor']
location_categories = list(set([o["category"] for o in locations]))
location_names = list(set([o["name"] for o in locations]))
manipulation_locations = list(set([o["name"] for o in locations if o["manipulation"] == "yes"]))

# hack
# ToDo: Rokus
most_probable_location_in_room_map = {
    "living_room" : "desk",
    "kitchen" : "kitchen_table",
    "bedroom" : "little_desk",
    "entrance" : "entrance_shelf",
    "balcony" : "bistro_table"
}


def get_location_from_room(room_id):
    if room_id in most_probable_location_in_room_map:
        return most_probable_location_in_room_map[room_id]
    return None

objects = [
    {"name": "curry", "category": "snack"},
    {"name": "plate", "category": "container"},
    {"name": "shampoo", "category": "cleaning_stuff"},
    {"name": "green_tea", "category": "drink"},
    {"name": "chewing_gum", "category": "snack"},
    {"name": "chopsticks", "category": "cutlery"},
    {"name": "soup_container", "category": "container"},
    {"name": "aquarius", "category": "drink"},
    {"name": "fries", "category": "snack"},
    {"name": "bowl", "category": "container"},
    {"name": "candy", "category": "snack"},
    {"name": "asience", "category": "cleaning_stuff"},
    {"name": "fork", "category": "cutlery"},
    {"name": "spoon", "category": "cutlery"},
    {"name": "hair_spray", "category": "cleaning_stuff"},
    {"name": "radish", "category": "food"},
    {"name": "apple", "category": "fruit"},
    {"name": "cold_brew", "category": "drink"},
    {"name": "onion", "category": "food"},
    {"name": "corn", "category": "food"},
    {"name": "jelly", "category": "snack"},
    {"name": "bread", "category": "food"},
    {"name": "cup_star", "category": "snack"},
    {"name": "orange", "category": "fruit"},
    {"name": "moisturizer", "category": "cleaning_stuff"},
    {"name": "coke", "category": "drink"}
]

object_names = list(set([o["name"] for o in objects]))
object_categories = list(set([o["category"] for o in objects]))

category_locations = {
    "container": {"left_rack": "on_top_of"},
    "cleaning_stuff": {"right_rack": "on_top_of"},
    "cutlery": {"sideboard": "on_top_of"},
    "food": {"kitchen_shelf": "shelf3"},
    "drink": {"kitchen_counter": "on_top_of"},
    "snack": {"kitchen_rack": "shelf2"},
    "fruit": {"bistro_table": "on_top_of"}
}

# "left_planks": [],
# "right_planks": [],
inspect_areas = {
    "kitchen_shelf": ["shelf2", "shelf3", "shelf4", "shelf5", "shelf6"],
    "balcony_shelf": ["shelf2", "shelf3", "shelf4"],
    "kitchen_rack": ["shelf2", "shelf3", "shelf4"],
    "kitchen_table": ["on_top_of", "on_top_of_back"],
    "sideboard": ["on_top_of", "shelf1"],
    "left_rack": ["on_top_of", "shelf3", "shelf2"],
    "right_rack": ["on_top_of", "shelf3", "shelf2"],
    "left_planks": ["on_top_of"],
    "right_planks": ["on_top_of"]
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
