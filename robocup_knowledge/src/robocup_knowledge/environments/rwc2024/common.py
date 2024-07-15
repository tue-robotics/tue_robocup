from robocup_knowledge.utils import *  # noqa: F401, F403

female_names = sorted(["sophie", "julia", "emma", "sara", "laura", "hayley", "susan", "fleur", "gabrielle", "robin", "jesse", "noah"])
male_names = sorted(["robin", "john", "liam", "lucas", "william", "kevin", "jesse", "noah", "harrie", "peter"])
names = set(female_names)
names.update(set(male_names))
names = sorted(list(names))

# rooms: hallway office kitchen living_room

locations = [
    {"name": "hallway_cabinet", "room": "hallway", "manipulation": True},
    {"name": "desk", "room": "office", "manipulation": True},
    {"name": "shelf", "room": "office", "manipulation": True},
    {"name": "coathanger", "room": "hallway", "manipulation": False},
    {"name": "tv_table", "room": "living_room", "manipulation": True},
    {"name": "lounge_chair", "room": "living_room", "manipulation": False},
    {"name": "lamp", "room": "living_room", "manipulation": False},
    {"name": "couch", "room": "living_room", "manipulation": False},
    {"name": "coffee_table", "room": "living_room", "manipulation": True},
    {"name": "trashcan", "room": "kitchen", "manipulation": False},
    {"name": "kitchen_cabinet", "room": "kitchen", "manipulation": True},
    {"name": "dinner_table", "room": "kitchen", "manipulation": True},
    {"name": "dishwasher", "room": "kitchen", "manipulation": True},
    {"name": "kitchen_counter", "room": "kitchen", "manipulation": True},
]

location_rooms = list(set([o["room"] for o in locations]))
location_names = list(set([o["name"] for o in locations]))
manipulation_locations = list(set([o["name"] for o in locations if o["manipulation"]]))

objects = [
    {"name": "soap", "category": "cleaning_supplies"},
    {"name": "dishwasher_tab", "category": "cleaning_supplies"},
    {"name": "washcloth", "category": "cleaning_supplies"},
    {"name": "sponge", "category": "cleaning_supplies"},
    {"name": "cola", "category": "drinks"},
    {"name": "ice_tea", "category": "drinks"},
    {"name": "water", "category": "drinks"},
    {"name": "milk", "category": "drinks"},
    {"name": "big_coke", "category": "drinks"},
    {"name": "fanta", "category": "drinks"},
    {"name": "dubbelfris", "category": "drinks"},
    {"name": "cornflakes", "category": "food"},
    {"name": "pea_soup", "category": "food"},
    {"name": "curry", "category": "food"},
    {"name": "pancake_mix", "category": "food"},
    {"name": "hagelslag", "category": "food"},
    {"name": "sausages", "category": "food"},
    {"name": "mayonaise", "category": "food"},
    {"name": "candle", "category": "decorations"},
    {"name": "pear", "category": "fruits"},
    {"name": "plum", "category": "fruits"},
    {"name": "peach", "category": "fruits"},
    {"name": "lemon", "category": "fruits"},
    {"name": "orange", "category": "fruits"},
    {"name": "strawberry", "category": "fruits"},
    {"name": "banana", "category": "fruits"},
    {"name": "apple", "category": "fruits"},
    {"name": "stroopwafel", "category": "snacks"},
    {"name": "candy", "category": "snacks"},
    {"name": "liquorice", "category": "snacks"},
    {"name": "crisps", "category": "snacks"},
    {"name": "pringles", "category": "snacks"},
    {"name": "tictac", "category": "snacks"},
    {"name": "spoon", "category": "dishes"},
    {"name": "plate", "category": "dishes"},
    {"name": "cup", "category": "dishes"},
    {"name": "fork", "category": "dishes"},
    {"name": "bowl", "category": "dishes"},
    {"name": "knife", "category": "dishes"},
]

object_names = list(set([o["name"] for o in objects]))
object_categories = list(set([o["category"] for o in objects]))

category_locations = {
    "decorations": {"desk": "on_top_of"},
    "cleaning_supplies": {"shelf": "on_top_of"},
    "toys": {"tv_table": "on_top_of"},
    "fruits": {"coffee_table": "on_top_of"},
    "drinks": {"kitchen_cabinet": "on_top_of"},
    "snacks": {"dinner_table": "on_top_of"},
    "dishes": {"dishwasher": "on_top_of"},
    "food": {"kitchen_counter": "on_top_of"},
}

inspect_areas = {
    "kitchen_cabinet": ["shelf2", "shelf3"],
}

inspect_positions = {
    "kitchen_cabinet": {
        # "shelf1_l": "in_front_of_l",
        # "shelf2_l": "in_front_of_l",
        # "shelf3_l": "in_front_of_l",
        # "shelf4_l": "in_front_of_l",
        # "shelf5_l": "in_front_of_l",
        # "shelf1_r": "in_front_of_r",
        # "shelf2_r": "in_front_of_r",
        "shelf3_r": "in_front_of_r",
        "shelf4_r": "in_front_of_r",
        # "shelf5_r": "in_front_of_r",
        # "on_top_of": "in_front_of",
    }
}

drink_spec = "T['drink': O] -> OPTIONS[O]\n\n"
for drink in [obj["name"] for obj in objects if obj["category"] == "drinks"]:
    drink_spec += "OPTIONS['{drink}'] -> {drink}\n".format(drink=drink)
