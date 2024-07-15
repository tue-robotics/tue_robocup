def is_location(location):
    for loc in locations:  # noqa: F821
        if loc["name"] == location:
            return True
    return False


def get_room(location):
    for loc in locations:  # noqa: F821
        if loc["name"] == location:
            return loc["room"]
    return None


def is_room(entity_id):
    return entity_id in location_rooms  # noqa: F821


def get_inspect_areas(location):
    if location in inspect_areas:  # noqa: F821
        return inspect_areas[location]  # noqa: F821
    else:
        return ["on_top_of"]


def get_inspect_position(location, area=""):
    if location in inspect_positions and area in inspect_positions[location]:  # noqa: F821
        return inspect_positions[location][area]  # noqa: F821
    else:
        return "in_front_of"


def is_manipulation_location(location):
    for loc in locations:  # noqa: F821
        if loc["name"] == location and loc["manipulation"]:
            return True
    return False


def get_locations(room=None, manipulation_location=None):
    return [loc["name"] for loc in locations  # noqa: F821
                if (room is None or loc["room"] == room) and \
                   (manipulation_location is None or manipulation_location == is_manipulation_location(loc["name"]))]


def is_known_object(obj):
    for o in objects:  # noqa: F821
        if o["name"] == obj:
            return True
    return False


def get_objects(category=None):
    return [obj["name"] for obj in objects  # noqa: F821
                if category is None or category == obj["category"]]


def get_object_category(obj):
    for o in objects:  # noqa: F821
        if o["name"] == obj:
            return o["category"]
    return None


def get_object_color(obj):
    for o in objects:  # noqa: F821
        if o["name"] == obj:
            return o["color"]
    return None


def get_object_size(obj):
    for o in objects:  # noqa: F821
        if o["name"] == obj:
            return o["volume"]
    return None


def get_object_weight(obj):
    for o in objects:  # noqa: F821
        if o["name"] == obj:
            return o["weight"]
    return None


# Returns (location, area_name)
def get_object_category_location(obj_cat):
    location, area_name = next(iter(category_locations[obj_cat].items()))  # noqa: F821
    return location, area_name
