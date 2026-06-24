# Inspired by https://stackoverflow.com/questions/9937279/can-modules-have-properties
# and https://stackoverflow.com/questions/880530/can-modules-have-properties-the-same-way-that-objects-can

import importlib
import os
import sys
from collections.abc import Iterable

# ToDo: add typing

EXPECTED_KNOWLEDGE = [
    "female_names",
    "male_names",
    "locations",
    "objects",
    "category_locations",
    "inspect_areas",
    "inspect_positions",
    "default_target_radius",
    "most_probable_location_in_room_map",
]


class KnowledgeFunctions(sys.__class__):
    def __init__(self):
        # ToDo: no hardcoding.

        robot_env = os.environ.get('ROBOT_ENV')
        common = importlib.import_module(".common", f"robocup_knowledge.environments.{robot_env}")

        # Add expected knowledge to self
        for key in EXPECTED_KNOWLEDGE:
            setattr(self, key, getattr(common, key))

    @property
    def names(self) -> Iterable:
        return self.female_names + self.male_names

    @property
    def location_rooms(self):
        return list(set([o["room"] for o in self.locations]))

    @property
    def rooms(self):  # Workshop is added in impuls/common.py
        return self.location_rooms

    @property
    def location_categories(self):
        return list(set([o["category"] for o in self.locations]))

    @property
    def location_names(self):
        return list(set([o["name"] for o in self.locations]))

    @property
    def object_names(self):
        return list(set([o["name"] for o in self.objects]))

    @property
    def object_categories(self):
        return list(set([o["category"] for o in self.objects]))

    @property
    def object_colors(self):  # This is present as "object_color" (singular) in impuls/common.py
        return list(set([o["color"] for o in self.objects]))

    @property
    def object_sizes(self):  # This is present as "object_size" (singular) in impuls/common
        return list(set([o["volume"] for o in self.objects]))

    @property
    def object_weights(self):  # This is present as "object_weight" (singular in impuls/common
        return list(set([o["weight"] for o in self.objects]))

    def get_location_from_room(self, room_id):
        if room_id in self.most_probable_location_in_room_map:
            return self.most_probable_location_in_room_map[room_id]
        return None

    def object_names_of_category(self, category):
        return [obj['name'] for obj in self.objects if obj['category'] == category]

    @property
    def drink_names(self):
        return self.object_names_of_category("drink")

    @property
    def drink_spec(self):
        result = "T['drink': O] -> OPTIONS[O]\n\n"
        for dn in self.drink_names:
            result += "OPTIONS['{drink}'] -> {drink}\n".format(drink=dn)
        return result

    def is_location(self, location):
        return any([loc["name"] == location for loc in self.locations])

    def get_room(self, location):
        for loc in self.locations:
            if loc["name"] == location:
                return loc["room"]
        return None

    def is_room(self, entity_id):
        return entity_id in self.rooms

    def get_inspect_areas(self, location):
        if location in self.inspect_areas:
            return self.inspect_areas[location]
        else:
            return ["on_top_of"]

    def get_inspect_position(self, location, area=""):  # Does this default argument make sense?
        if location in self.inspect_positions and area in self.inspect_positions[location]:
            return self.inspect_positions[location][area]  # Is this a list or a string?
        return "in_front_of"

    def is_pick_location(self, location):
        return any([l["name"] == location and l["manipulation"] == "yes" for l in self.locations])

    def is_place_location(self, location):
        return any([l["name"] == location and l["manipulation"] in ["yes", "only_putting"] for l in self.locations])

    def get_locations(self, room=None, pick_location=None, place_location=None):
        raise AssertionError("This doesn't make any sense")  # I don't know what this is supposed to do

    def is_known_object(self, obj):
        return any([o["name"] == obj for o in self.objects])

    def get_objects(self, category=None):
        return [o["name"] for o in self.objects if category is None or o["category"] == category]

    def get_object_category(self, obj):
        for o in self.objects:
            if o["name"] == obj:
                return o["category"]
        return None

    def get_object_color(self, obj):
        for o in self.objects:
            if o["name"] == obj:
                return o["color"]
        return None

    def get_object_size(self, obj):
        for o in self.objects:
            if o["name"] == obj:
                return o["volume"]
        return None

    def get_object_weight(self, obj):
        for o in self.objects:
            if o["name"] == obj:
                return o["weight"]
        return None

    def get_object_category_location(self, obj_cat):
        return next(iter(self.category_locations[obj_cat].items()))


sys.modules[__name__] = KnowledgeFunctions()
