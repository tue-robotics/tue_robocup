# TODO: update this from the common knowledge

from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

rooms = ["kitchen", "bedroom", "living_room"]

object_aliases = {"dinner table" : "dinnertable"}

# mapping from furniture to small objects that are on top of them (and can be grabbed)
furniture_to_objects = {
    "cabinet": [
        "beer",
        "bifrutas",
        "coffee_pads",
        "coke",
        "deodorant"
    ],

    "dinnertable": [
        "fanta",
        "ice_tea",
        "mentos",
        "sprite",
        "tea",
        "teddy_bear",
        "water",
        "xylit24_spearmint",
        "xylit24_white"],

    "bed": []
}
